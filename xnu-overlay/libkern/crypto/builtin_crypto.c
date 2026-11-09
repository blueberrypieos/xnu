/*
 * SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2026 The Blueberry Pie OS Authors
 */

#include <kern/startup.h>
#include <kern/locks.h>
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <machine/machine_routines.h>
#include <machine/atomic.h>
#include <prng/random.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <libkern/libkern.h>
#include <libkern/crypto/register_crypto.h>
#include <libkern/crypto/rand.h>
#include <corecrypto/cc.h>
#include <corecrypto/ccdigest.h>
#include <corecrypto/cchmac.h>
#include <corecrypto/ccmd5.h>
#include <corecrypto/ccsha1.h>
#include <corecrypto/ccsha2.h>
#include <corecrypto/ccdrbg.h>
#include <corecrypto/cckprng.h>
#include <corecrypto/ccrng.h>
#include <corecrypto/ccmode.h>
#include <corecrypto/ccaes.h>
#include <corecrypto/ccdes.h>
#include <corecrypto/ccpad.h>
#include <corecrypto/ccchacha20poly1305.h>
#include <corecrypto/ccrsa.h>
#include <libkern/crypto/aes.h>
#include <libkern/crypto/aesxts.h>
#include <libkern/crypto/des.h>

extern int get_preemption_level(void);
extern void read_random(void *buffer, u_int numbytes);
extern bool pc_cpu_has_aes(void);
extern void pc_cpu_use_hw(bool use);

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#pragma mark Digests

#pragma clang diagnostic ignored "-Wcast-align"
#pragma clang diagnostic ignored "-Wunused-parameter"

static void
builtin_ccdigest_init(const struct ccdigest_info *di, ccdigest_ctx_t ctx)
{
	ccdigest_init(di, ctx);
}

static void
builtin_ccdigest_update(const struct ccdigest_info *di, ccdigest_ctx_t ctx,
    unsigned long len, const void *data)
{
	ccdigest_update(di, ctx, len, data);
}

static void
builtin_ccdigest_final(const struct ccdigest_info *di, ccdigest_ctx_t ctx, void *digest)
{
	di->final(di, ctx, digest);
}

static void
builtin_ccdigest(const struct ccdigest_info *di, unsigned long len,
    const void *data, void *digest)
{
	ccdigest_di_decl(di, ctx);
	ccdigest_init(di, ctx);
	ccdigest_update(di, ctx, len, data);
	di->final(di, ctx, digest);
	ccdigest_di_clear(di, ctx);
}

static void
builtin_cchmac_init(const struct ccdigest_info *di, cchmac_ctx_t ctx,
    unsigned long key_len, const void *key)
{
	cchmac_init(di, ctx, key_len, key);
}

static void
builtin_cchmac_update(const struct ccdigest_info *di, cchmac_ctx_t ctx,
    unsigned long data_len, const void *data)
{
	cchmac_update(di, ctx, data_len, data);
}

static void
builtin_cchmac_final(const struct ccdigest_info *di, cchmac_ctx_t ctx,
    unsigned char *mac)
{
	cchmac_final(di, ctx, mac);
}

static void
builtin_cchmac(const struct ccdigest_info *di, unsigned long key_len,
    const void *key, unsigned long data_len, const void *data,
    unsigned char *mac)
{
	cchmac(di, key_len, key, data_len, data, mac);
}

#pragma mark Generic digest and HMAC interface

#define MAX_DIGEST_SIZE 64

static const struct ccdigest_info *
alg_di(crypto_digest_alg_t alg)
{
	switch (alg) {
	case CRYPTO_DIGEST_ALG_MD5:
		return ccmd5_di();
	case CRYPTO_DIGEST_ALG_SHA1:
		return ccsha1_di();
	case CRYPTO_DIGEST_ALG_SHA256:
		return ccsha256_di();
	case CRYPTO_DIGEST_ALG_SHA384:
		return ccsha384_di();
	case CRYPTO_DIGEST_ALG_SHA512:
		return ccsha512_di();
	default:
		panic("builtin_crypto: unsupported digest algorithm %u", alg);
	}
}

static void
copy_truncated(const uint8_t *full, size_t full_size, void *out, size_t out_size)
{
	if (out_size > full_size) {
		panic("builtin_crypto: digest size %zu > %zu", out_size, full_size);
	}
	memcpy(out, full, out_size);
}

static size_t
builtin_digest_ctx_size(crypto_digest_alg_t alg)
{
	return ccdigest_di_size(alg_di(alg));
}

static void
builtin_digest_init(crypto_digest_alg_t alg, void *ctx, size_t ctx_size)
{
	const struct ccdigest_info *di = alg_di(alg);

	assert(ctx_size >= ccdigest_di_size(di));
	ccdigest_init(di, ctx);
}

static void
builtin_digest_update(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    const void *data, size_t data_size)
{
	ccdigest_update(alg_di(alg), ctx, data_size, data);
}

static void
builtin_digest_final(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    void *digest, size_t digest_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];

	di->final(di, ctx, full);
	copy_truncated(full, di->output_size, digest, digest_size);
	cc_clear(sizeof(full), full);
}

static void
builtin_digest(crypto_digest_alg_t alg, const void *data, size_t data_size,
    void *digest, size_t digest_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];

	builtin_ccdigest(di, data_size, data, full);
	copy_truncated(full, di->output_size, digest, digest_size);
	cc_clear(sizeof(full), full);
}

static size_t
builtin_hmac_ctx_size(crypto_digest_alg_t alg)
{
	return cchmac_di_size(alg_di(alg));
}

static void
builtin_hmac_init(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    const void *key, size_t key_size)
{
	const struct ccdigest_info *di = alg_di(alg);

	assert(ctx_size >= cchmac_di_size(di));
	cchmac_init(di, ctx, key_size, key);
}

static void
builtin_hmac_update(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    const void *data, size_t data_size)
{
	cchmac_update(alg_di(alg), ctx, data_size, data);
}

static void
builtin_hmac_final_generate(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    void *tag, size_t tag_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];

	cchmac_final(di, ctx, full);
	copy_truncated(full, di->output_size, tag, tag_size);
	cc_clear(sizeof(full), full);
}

static bool
builtin_hmac_final_verify(crypto_digest_alg_t alg, void *ctx, size_t ctx_size,
    const void *tag, size_t tag_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];
	bool ok;

	cchmac_final(di, ctx, full);
	ok = tag_size <= di->output_size && cc_cmp_safe(tag_size, full, tag) == 0;
	cc_clear(sizeof(full), full);
	return ok;
}

static void
builtin_hmac_generate(crypto_digest_alg_t alg, const void *key, size_t key_size,
    const void *data, size_t data_size, void *tag, size_t tag_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];

	cchmac(di, key_size, key, data_size, data, full);
	copy_truncated(full, di->output_size, tag, tag_size);
	cc_clear(sizeof(full), full);
}

static bool
builtin_hmac_verify(crypto_digest_alg_t alg, const void *key, size_t key_size,
    const void *data, size_t data_size, const void *tag, size_t tag_size)
{
	const struct ccdigest_info *di = alg_di(alg);
	uint8_t full[MAX_DIGEST_SIZE];
	bool ok;

	cchmac(di, key_size, key, data_size, data, full);
	ok = tag_size <= di->output_size && cc_cmp_safe(tag_size, full, tag) == 0;
	cc_clear(sizeof(full), full);
	return ok;
}

#pragma mark Kernel PRNG

#define KPRNG_STATE_SIZE        512
#define KPRNG_REFRESH_NS        NSEC_PER_SEC

static LCK_GRP_DECLARE(builtin_crypto_grp, "builtin_crypto");
static LCK_SPIN_DECLARE(kprng_lock, &builtin_crypto_grp);

static struct {
	struct ccdrbg_info info;
	struct ccdrbg_nisthmac_custom custom;
	uint8_t state[KPRNG_STATE_SIZE] __attribute__((aligned(16)));
	cckprng_getentropy getentropy;
	void *getentropy_arg;
	uint64_t refresh_interval;
	uint64_t last_refresh;
} kprng;

static boolean_t
kprng_lock_acquire(void)
{
	boolean_t istate = ml_set_interrupts_enabled(FALSE);

	lck_spin_lock_grp(&kprng_lock, &builtin_crypto_grp);
	return istate;
}

static void
kprng_lock_release(boolean_t istate)
{
	lck_spin_unlock(&kprng_lock);
	ml_set_interrupts_enabled(istate);
}

static void
kprng_init_with_getentropy(struct cckprng_ctx *ctx, unsigned max_ngens,
    size_t seed_nbytes, const void *seed, size_t nonce_nbytes, const void *nonce,
    cckprng_getentropy getentropy, void *getentropy_arg)
{
	static const char ps[] = "xnu builtin kprng";
	int rc;

	kprng.custom.di = ccsha256_di();
	kprng.custom.strictFIPS = 0;
	ccdrbg_factory_nisthmac(&kprng.info, &kprng.custom);
	if (kprng.info.size > sizeof(kprng.state)) {
		panic("builtin_crypto: DRBG state %zu > %zu", kprng.info.size, sizeof(kprng.state));
	}
	rc = ccdrbg_init(&kprng.info, (struct ccdrbg_state *)kprng.state,
	    seed_nbytes, seed, nonce_nbytes, nonce, sizeof(ps) - 1, ps);
	if (rc != CCDRBG_STATUS_OK) {
		panic("builtin_crypto: ccdrbg_init() returned %d", rc);
	}
	kprng.getentropy = getentropy;
	kprng.getentropy_arg = getentropy_arg;
	nanoseconds_to_absolutetime(KPRNG_REFRESH_NS, &kprng.refresh_interval);
	kprng.last_refresh = mach_absolute_time();
}

static void
kprng_init(struct cckprng_ctx *ctx, size_t seed_nbytes, const void *seed,
    size_t nonce_nbytes, const void *nonce, cckprng_getentropy getentropy,
    void *getentropy_arg)
{
	kprng_init_with_getentropy(ctx, MAX_CPUS, seed_nbytes, seed, nonce_nbytes,
	    nonce, getentropy, getentropy_arg);
}

static void
kprng_initgen(struct cckprng_ctx *ctx, unsigned gen_idx)
{
}

static void
kprng_reseed(struct cckprng_ctx *ctx, size_t nbytes, const void *seed)
{
	boolean_t istate = kprng_lock_acquire();
	int rc = ccdrbg_reseed(&kprng.info, (struct ccdrbg_state *)kprng.state,
	    nbytes, seed, 0, NULL);
	kprng_lock_release(istate);
	if (rc != CCDRBG_STATUS_OK) {
		panic("builtin_crypto: ccdrbg_reseed() returned %d", rc);
	}
}

static void
kprng_refresh(struct cckprng_ctx *ctx)
{
	uint8_t entropy[CCKPRNG_ENTROPY_SIZE];
	size_t entropy_size = sizeof(entropy);
	uint64_t now = mach_absolute_time();

	if (ml_at_interrupt_context() || get_preemption_level() != 0 ||
	    !ml_get_interrupts_enabled() ||
	    now - os_atomic_load(&kprng.last_refresh, relaxed) < kprng.refresh_interval) {
		return;
	}
	os_atomic_store(&kprng.last_refresh, now, relaxed);

	if (kprng.getentropy(&entropy_size, entropy, kprng.getentropy_arg) > 0 &&
	    entropy_size > 0) {
		kprng_reseed(ctx, entropy_size, entropy);
	}
	cc_clear(sizeof(entropy), entropy);
}

static void
kprng_generate(struct cckprng_ctx *ctx, unsigned gen_idx, size_t nbytes, void *out)
{
	uint8_t *p = out;

	while (nbytes > 0) {
		size_t n = MIN(nbytes, CCKPRNG_MAX_REQUEST_SIZE);
		boolean_t istate = kprng_lock_acquire();
		int rc = ccdrbg_generate(&kprng.info, (struct ccdrbg_state *)kprng.state,
		    n, p, 0, NULL);
		kprng_lock_release(istate);
		if (rc != CCDRBG_STATUS_OK) {
			panic("builtin_crypto: ccdrbg_generate() returned %d", rc);
		}
		p += n;
		nbytes -= n;
	}
}

static const struct cckprng_funcs builtin_kprng_funcs = {
	.init = kprng_init,
	.initgen = kprng_initgen,
	.reseed = kprng_reseed,
	.refresh = kprng_refresh,
	.generate = kprng_generate,
	.init_with_getentropy = kprng_init_with_getentropy,
};

#pragma mark ccrng and crypto_random

static int
builtin_rng_generate(struct ccrng_state *rng, size_t outlen, void *out)
{
	uint8_t *p = out;

	while (outlen > 0) {
		u_int n = (u_int)MIN(outlen, (size_t)UINT_MAX);
		read_random(p, n);
		p += n;
		outlen -= n;
	}
	return 0;
}

static struct ccrng_state builtin_rng = {
	.generate = builtin_rng_generate,
};

static struct ccrng_state *
builtin_ccrng(int *error)
{
	if (error) {
		*error = 0;
	}
	return &builtin_rng;
}

struct kmem_rng {
	uint64_t s[4];
};

static uint64_t
kmem_rng_next(struct kmem_rng *r)
{
	uint64_t *s = r->s;
	uint64_t result = ((s[1] * 5) << 7 | (s[1] * 5) >> 57) * 9;
	uint64_t t = s[1] << 17;

	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];
	s[2] ^= t;
	s[3] = (s[3] << 45) | (s[3] >> 19);
	return result;
}

static size_t
builtin_random_kmem_ctx_size(void)
{
	return sizeof(struct kmem_rng);
}

static void
builtin_random_kmem_init(crypto_random_ctx_t ctx)
{
	struct kmem_rng *r = ctx;

	do {
		read_random(r->s, sizeof(r->s));
	} while ((r->s[0] | r->s[1] | r->s[2] | r->s[3]) == 0);
}

static void
builtin_random_generate(crypto_random_ctx_t ctx, void *random, size_t random_size)
{
	uint8_t *p = random;

	if (ctx == NULL) {
		builtin_rng_generate(&builtin_rng, random_size, random);
		return;
	}
	while (random_size > 0) {
		uint64_t v = kmem_rng_next(ctx);
		size_t n = MIN(random_size, sizeof(v));
		memcpy(p, &v, n);
		p += n;
		random_size -= n;
	}
}

static void
builtin_random_uniform(crypto_random_ctx_t ctx, uint64_t bound, uint64_t *random)
{
	uint64_t threshold, v;

	if (bound <= 1) {
		*random = 0;
		return;
	}
	threshold = (0 - bound) % bound;
	do {
		builtin_random_generate(ctx, &v, sizeof(v));
	} while (v < threshold);
	*random = v % bound;
}

#pragma mark Registration

static struct crypto_functions builtin_crypto_funcs = {
	.ccdigest_init_fn = builtin_ccdigest_init,
	.ccdigest_update_fn = builtin_ccdigest_update,
	.ccdigest_final_fn = builtin_ccdigest_final,
	.ccdigest_fn = builtin_ccdigest,
	.cchmac_init_fn = builtin_cchmac_init,
	.cchmac_update_fn = builtin_cchmac_update,
	.cchmac_final_fn = builtin_cchmac_final,
	.cchmac_fn = builtin_cchmac,
	.ccgcm_init_with_iv_fn = ccgcm_init_with_iv,
	.ccgcm_inc_iv_fn = ccgcm_inc_iv,
	.ccdes_key_is_weak_fn = ccdes_key_is_weak,
	.ccdes_key_set_odd_parity_fn = ccdes_key_set_odd_parity,
	.ccpad_cts3_encrypt_fn = ccpad_cts3_encrypt,
	.ccpad_cts3_decrypt_fn = ccpad_cts3_decrypt,
	.ccrsa_make_pub_fn = ccrsa_make_pub,
	.ccrsa_verify_pkcs1v15_fn = ccrsa_verify_pkcs1v15,
	.ccrng_fn = builtin_ccrng,
	.random_generate_fn = builtin_random_generate,
	.random_uniform_fn = builtin_random_uniform,
	.random_kmem_ctx_size_fn = builtin_random_kmem_ctx_size,
	.random_kmem_init_fn = builtin_random_kmem_init,
	.digest_ctx_size_fn = builtin_digest_ctx_size,
	.digest_init_fn = builtin_digest_init,
	.digest_update_fn = builtin_digest_update,
	.digest_final_fn = builtin_digest_final,
	.digest_fn = builtin_digest,
	.hmac_ctx_size_fn = builtin_hmac_ctx_size,
	.hmac_init_fn = builtin_hmac_init,
	.hmac_update_fn = builtin_hmac_update,
	.hmac_final_generate_fn = builtin_hmac_final_generate,
	.hmac_final_verify_fn = builtin_hmac_final_verify,
	.hmac_generate_fn = builtin_hmac_generate,
	.hmac_verify_fn = builtin_hmac_verify,
};

static const struct ccchacha20poly1305_fns builtin_chacha20poly1305_fns = {
	.info = ccchacha20poly1305_info,
	.init = ccchacha20poly1305_init,
	.reset = ccchacha20poly1305_reset,
	.setnonce = ccchacha20poly1305_setnonce,
	.incnonce = ccchacha20poly1305_incnonce,
	.aad = ccchacha20poly1305_aad,
	.encrypt = ccchacha20poly1305_encrypt,
	.finalize = ccchacha20poly1305_finalize,
	.decrypt = ccchacha20poly1305_decrypt,
	.verify = ccchacha20poly1305_verify,
};

__startup_func
static void
builtin_crypto_fill_modes(struct crypto_functions *f)
{
	f->ccaes_ecb_encrypt = ccaes_ecb_encrypt_mode();
	f->ccaes_ecb_decrypt = ccaes_ecb_decrypt_mode();
	f->ccaes_cbc_encrypt = ccaes_cbc_encrypt_mode();
	f->ccaes_cbc_decrypt = ccaes_cbc_decrypt_mode();
	f->ccaes_ctr_crypt = ccaes_ctr_crypt_mode();
	f->ccaes_xts_encrypt = ccaes_xts_encrypt_mode();
	f->ccaes_xts_decrypt = ccaes_xts_decrypt_mode();
	f->ccaes_gcm_encrypt = ccaes_gcm_encrypt_mode();
	f->ccaes_gcm_decrypt = ccaes_gcm_decrypt_mode();
	f->ccchacha20poly1305_fns = &builtin_chacha20poly1305_fns;
	f->ccdes_ecb_encrypt = ccdes_ecb_encrypt_mode();
	f->ccdes_ecb_decrypt = ccdes_ecb_decrypt_mode();
	f->ccdes_cbc_encrypt = ccdes_cbc_encrypt_mode();
	f->ccdes_cbc_decrypt = ccdes_cbc_decrypt_mode();
	f->cctdes_ecb_encrypt = ccdes3_ecb_encrypt_mode();
	f->cctdes_ecb_decrypt = ccdes3_ecb_decrypt_mode();
	f->cctdes_cbc_encrypt = ccdes3_cbc_encrypt_mode();
	f->cctdes_cbc_decrypt = ccdes3_cbc_decrypt_mode();
}

#pragma mark Known-answer tests

static void
kat_check(const char *what, const void *got, const void *want, size_t len)
{
	if (memcmp(got, want, len) != 0) {
		panic("builtin_crypto: %s known-answer test failed", what);
	}
}

__startup_func
static void
builtin_crypto_self_test(void)
{
	static const uint8_t md5_abc[] = {
		0x90, 0x01, 0x50, 0x98, 0x3c, 0xd2, 0x4f, 0xb0, 0xd6, 0x96, 0x3f, 0x7d, 0x28, 0xe1, 0x7f, 0x72,
	};
	static const uint8_t sha1_abc[] = {
		0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a, 0xba, 0x3e, 0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c,
		0x9c, 0xd0, 0xd8, 0x9d,
	};
	static const uint8_t sha256_abc[] = {
		0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
		0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
	};
	static const uint8_t sha384_abc[] = {
		0xcb, 0x00, 0x75, 0x3f, 0x45, 0xa3, 0x5e, 0x8b, 0xb5, 0xa0, 0x3d, 0x69, 0x9a, 0xc6, 0x50, 0x07,
		0x27, 0x2c, 0x32, 0xab, 0x0e, 0xde, 0xd1, 0x63, 0x1a, 0x8b, 0x60, 0x5a, 0x43, 0xff, 0x5b, 0xed,
		0x80, 0x86, 0x07, 0x2b, 0xa1, 0xe7, 0xcc, 0x23, 0x58, 0xba, 0xec, 0xa1, 0x34, 0xc8, 0x25, 0xa7,
	};
	uint8_t out[48];

	builtin_digest(CRYPTO_DIGEST_ALG_MD5, "abc", 3, out, sizeof(md5_abc));
	kat_check("MD5", out, md5_abc, sizeof(md5_abc));
	builtin_digest(CRYPTO_DIGEST_ALG_SHA1, "abc", 3, out, sizeof(sha1_abc));
	kat_check("SHA-1", out, sha1_abc, sizeof(sha1_abc));
	builtin_digest(CRYPTO_DIGEST_ALG_SHA256, "abc", 3, out, sizeof(sha256_abc));
	kat_check("SHA-256", out, sha256_abc, sizeof(sha256_abc));
	builtin_digest(CRYPTO_DIGEST_ALG_SHA384, "abc", 3, out, sizeof(sha384_abc));
	kat_check("SHA-384", out, sha384_abc, sizeof(sha384_abc));
}

static const uint8_t kat_ctr_ct[] = {
	0x26, 0xe4, 0x81, 0xa1, 0x78, 0x1d, 0x63, 0x1d, 0xcf, 0x0a, 0x80, 0x66, 0x57, 0x71, 0xc7, 0xc0,
	0xc2, 0xf2, 0xa1, 0x79, 0xcb, 0xe1, 0xbe, 0x28, 0x2c, 0x26, 0xfd, 0x2a, 0xfa, 0x0b, 0x85, 0x77,
	0x72, 0xd2, 0x34, 0xff, 0xd0,
};
static const uint8_t kat_xts_key[] = {
	0x10, 0x17, 0x1e, 0x25, 0x2c, 0x33, 0x3a, 0x41, 0x48, 0x4f, 0x56, 0x5d, 0x64, 0x6b, 0x72, 0x79,
	0x80, 0x87, 0x8e, 0x95, 0x9c, 0xa3, 0xaa, 0xb1, 0xb8, 0xbf, 0xc6, 0xcd, 0xd4, 0xdb, 0xe2, 0xe9,
};
static const uint8_t kat_xts_iv[] = {
	0x33, 0x38, 0x3d, 0x42, 0x47, 0x4c, 0x51, 0x56, 0x5b, 0x60, 0x65, 0x6a, 0x6f, 0x74, 0x79, 0x7e,
};
static const uint8_t kat_xts_ct[] = {
	0x4d, 0x6b, 0xe1, 0xab, 0x6a, 0xcb, 0x65, 0x8d, 0xfd, 0x9a, 0x33, 0x7a, 0x5e, 0xa9, 0x06, 0x6d,
	0x61, 0xc3, 0x4c, 0xad, 0x58, 0x60, 0x13, 0xbe, 0x0d, 0xf5, 0xba, 0xb4, 0xaf, 0xb2, 0x54, 0xd9,
};
static const uint8_t kat_gcm_iv0[] = {
	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab,
};
static const uint8_t kat_gcm_iv1[] = {
	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xac,
};
static const uint8_t kat_gcm_aad[] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
};
static const uint8_t kat_gcm_ct[] = {
	0xbc, 0xca, 0x07, 0x0c, 0x20, 0xae, 0x7e, 0xb5, 0xa4, 0xec, 0x26, 0xd1, 0x08, 0x8e, 0xed, 0x95,
	0x06, 0x07, 0x05, 0x5a, 0x81, 0x84, 0xbf, 0x65, 0x45, 0x1e, 0x5f, 0xc2, 0x85, 0xfd, 0xcb, 0xd4,
	0xf6, 0x27, 0xcc, 0x3c, 0x4d,
};
static const uint8_t kat_gcm_tag[] = {
	0x79, 0xb2, 0x45, 0xa3, 0x47, 0xc6, 0xcb, 0x9c, 0xbc, 0xcd, 0x96, 0x3d, 0xcc, 0xdf, 0x2d, 0xdb,
};
static const uint8_t kat_chacha_nonce[] = {
	0x07, 0x12, 0x1d, 0x28, 0x33, 0x3e, 0x49, 0x54, 0x5f, 0x6a, 0x75, 0x80,
};
static const uint8_t kat_chacha_ct[] = {
	0xe1, 0x19, 0xc5, 0xa4, 0x8e, 0x9a, 0xcc, 0x51, 0x00, 0x5c, 0x57, 0x7b, 0x2b, 0xeb, 0x6d, 0xfd,
	0xc1, 0x9e, 0xc5, 0x78, 0x32, 0xa0, 0xd3, 0x5e, 0xbb, 0xfb, 0x1e, 0x8b, 0x67, 0x09, 0x34, 0xaa,
	0x4c, 0x7b, 0xa4, 0xcd, 0x9b,
};
static const uint8_t kat_chacha_tag[] = {
	0x3f, 0xea, 0xc8, 0x8f, 0x55, 0xf6, 0xa3, 0x80, 0x5e, 0x64, 0xfd, 0x0c, 0x97, 0x14, 0x69, 0x55,
};
static const uint8_t kat_des_key[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t kat_des_ct[] = {
	0xcf, 0x44, 0xa9, 0xd4, 0x00, 0x04, 0x58, 0xe1,
};
static const uint8_t kat_des3_key[] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
	0x89, 0xab, 0xcd, 0xef, 0x01, 0x23, 0x45, 0x67,
};
static const uint8_t kat_des3_ct[] = {
	0x73, 0x12, 0x49, 0x8c, 0x82, 0x96, 0x96, 0x56, 0xbd, 0x7b, 0x22, 0xf8, 0x42, 0x0a, 0x8f, 0xe8,
};
static const uint8_t kat_cts_ct[] = {
	0x09, 0x4e, 0x49, 0x8d, 0x71, 0x96, 0xf2, 0x20, 0x57, 0xf7, 0x8f, 0x13, 0x0b, 0xcf, 0xf6, 0x23,
	0xe3, 0x07, 0xd0, 0x6b,
};
static const uint8_t kat_rsa_n[] = {
	0xb8, 0x6e, 0x46, 0xcf, 0xa4, 0x39, 0xa4, 0x12, 0xaa, 0xfa, 0xc9, 0x45, 0xa9, 0xc3, 0xa4, 0x83,
	0xe2, 0x32, 0xd3, 0x95, 0x8c, 0x91, 0xb4, 0x7e, 0x17, 0xb7, 0xe7, 0x21, 0x85, 0xea, 0x08, 0x28,
	0x06, 0x04, 0x87, 0xb9, 0x9c, 0x19, 0xab, 0x2b, 0x13, 0x7a, 0x6a, 0xb6, 0x8a, 0x36, 0x08, 0x9e,
	0x1f, 0x53, 0xfd, 0xa3, 0x33, 0xca, 0x20, 0xa5, 0x1a, 0x71, 0xe9, 0x9a, 0x47, 0xd0, 0x00, 0xc7,
	0x91, 0xa1, 0x39, 0x4c, 0x07, 0xc2, 0x52, 0x7e, 0x4c, 0x2d, 0xde, 0x70, 0x50, 0x6d, 0xb2, 0xa9,
	0xb2, 0x92, 0xe0, 0xa8, 0xad, 0x64, 0xcd, 0x01, 0x11, 0x4c, 0x99, 0x27, 0xf6, 0xa2, 0x2f, 0x5c,
	0x06, 0x3f, 0x46, 0x08, 0xc4, 0x39, 0x69, 0x79, 0x62, 0xeb, 0x04, 0xa5, 0x79, 0xfd, 0x43, 0xcc,
	0x59, 0xa7, 0x4e, 0x7a, 0x73, 0x2d, 0xda, 0x53, 0x45, 0x90, 0x64, 0x89, 0x94, 0xbb, 0xf2, 0x81,
};
static const uint8_t kat_rsa_sig[] = {
	0x17, 0xc9, 0xdc, 0x6b, 0xac, 0xf6, 0x61, 0x4f, 0x33, 0x69, 0x57, 0x24, 0xe4, 0xa3, 0xd6, 0x82,
	0xa3, 0x7a, 0x9c, 0x0a, 0x9d, 0xe8, 0x4b, 0x6e, 0xf7, 0x80, 0xbe, 0x30, 0x86, 0xc1, 0x9a, 0xb2,
	0xdc, 0x8d, 0x91, 0xcc, 0xc5, 0xb9, 0xc6, 0xc3, 0x27, 0x9e, 0xc1, 0xc6, 0xda, 0xfc, 0x50, 0x4c,
	0xf8, 0x5b, 0x1b, 0xff, 0x01, 0x4b, 0x96, 0x5d, 0xb9, 0xb3, 0xc1, 0x24, 0x08, 0xb3, 0x33, 0xc9,
	0x40, 0x46, 0xca, 0x79, 0x25, 0xe2, 0x1a, 0x82, 0x88, 0xc6, 0xe8, 0x16, 0xc8, 0x9d, 0xb2, 0xc6,
	0xf6, 0x38, 0xfd, 0xd4, 0x62, 0x21, 0x31, 0xc5, 0x33, 0x78, 0x15, 0xff, 0xa4, 0x2f, 0xdf, 0x19,
	0xde, 0xd3, 0xd7, 0x3c, 0xd0, 0xcd, 0x77, 0x5e, 0x78, 0x6a, 0x31, 0xac, 0x37, 0xac, 0x4f, 0xa0,
	0xfe, 0x02, 0x90, 0x74, 0xc8, 0x39, 0x87, 0x5e, 0x6e, 0x4b, 0x0a, 0x8f, 0xaf, 0x6e, 0x6b, 0x7c,
};

#define KAT_CTX 2048

static void
kat_seq(uint8_t *p, size_t n, uint8_t start, uint8_t step)
{
	for (size_t i = 0; i < n; i++) {
		p[i] = (uint8_t)(start + i * step);
	}
}

__startup_func
static void
builtin_crypto_cipher_self_test(const struct crypto_functions *f)
{
	static const uint8_t aes_pt[] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
	};
	static const uint8_t aes128_ct[] = {
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
	};
	static const uint8_t aes256_ct[] = {
		0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89,
	};
	static uint8_t ctx[KAT_CTX] __attribute__((aligned(16))), ctx2[KAT_CTX] __attribute__((aligned(16)));
	uint8_t key[32], pt[40], out[64], back[64], iv[16], tag[16];
	bool valid;

	if (f->ccaes_cbc_encrypt->size > sizeof(aes_encrypt_ctx) || f->ccaes_cbc_decrypt->size > sizeof(aes_decrypt_ctx) ||
	    f->ccaes_xts_encrypt->size > sizeof(((symmetric_xts *)0)->enc) ||
	    f->ccaes_xts_decrypt->size > sizeof(((symmetric_xts *)0)->dec) ||
	    f->ccdes_ecb_encrypt->size > sizeof(((des_ecb_key_schedule *)0)->enc) ||
	    f->cctdes_ecb_encrypt->size > sizeof(((des3_ecb_key_schedule *)0)->enc)) {
		panic("builtin_crypto: a mode's context is larger than libkern's");
	}

	kat_seq(key, sizeof(key), 0, 1);
	kat_seq(pt, 37, 0x40, 3);

	const struct {
		size_t key_len;
		const uint8_t *ct;
	} aes[] = { { 16, aes128_ct }, { 32, aes256_ct } };
	for (size_t i = 0; i < sizeof(aes) / sizeof(aes[0]); i++) {
		const struct ccmode_ecb *e = f->ccaes_ecb_encrypt, *d = f->ccaes_ecb_decrypt;
		assert(e->size <= KAT_CTX && d->size <= KAT_CTX);
		e->init(e, (ccecb_ctx *)ctx, aes[i].key_len, key);
		e->ecb((ccecb_ctx *)ctx, 1, aes_pt, out);
		kat_check("AES-ECB encrypt", out, aes[i].ct, 16);
		d->init(d, (ccecb_ctx *)ctx, aes[i].key_len, key);
		d->ecb((ccecb_ctx *)ctx, 1, out, back);
		kat_check("AES-ECB decrypt", back, aes_pt, 16);
	}

	{
		const struct ccmode_cbc *e = f->ccaes_cbc_encrypt, *d = f->ccaes_cbc_decrypt;
		kat_seq(iv, 16, 0xf0, 1);
		cccbc_init(e, (cccbc_ctx *)ctx, 16, key);
		cccbc_set_iv(e, (cccbc_iv *)ctx2, iv);
		cccbc_update(e, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 2, pt, out);
		cccbc_init(d, (cccbc_ctx *)ctx, 16, key);
		cccbc_set_iv(d, (cccbc_iv *)ctx2, iv);
		cccbc_update(d, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 2, out, back);
		kat_check("AES-CBC", back, pt, 32);
	}

	{
		const struct ccmode_ctr *m = f->ccaes_ctr_crypt;
		assert(m->size <= KAT_CTX);
		kat_seq(iv, 16, 0xf0, 1);
		m->init(m, (ccctr_ctx *)ctx, 16, key, iv);
		m->ctr((ccctr_ctx *)ctx, 37, pt, out);
		kat_check("AES-CTR", out, kat_ctr_ct, sizeof(kat_ctr_ct));
	}

	{
		const struct ccmode_xts *e = f->ccaes_xts_encrypt, *d = f->ccaes_xts_decrypt;
		uint8_t tweak[64] __attribute__((aligned(16)));
		kat_seq(back, 32, 0x80, 1);
		assert(e->size <= KAT_CTX && e->tweak_size <= sizeof(tweak));
		e->init(e, (ccxts_ctx *)ctx, 16, kat_xts_key, kat_xts_key + 16);
		e->set_tweak((ccxts_ctx *)ctx, (ccxts_tweak *)tweak, kat_xts_iv);
		e->xts((ccxts_ctx *)ctx, (ccxts_tweak *)tweak, 2, back, out);
		kat_check("AES-XTS encrypt", out, kat_xts_ct, sizeof(kat_xts_ct));
		d->init(d, (ccxts_ctx *)ctx, 16, kat_xts_key, kat_xts_key + 16);
		d->set_tweak((ccxts_ctx *)ctx, (ccxts_tweak *)tweak, kat_xts_iv);
		d->xts((ccxts_ctx *)ctx, (ccxts_tweak *)tweak, 2, out, back);
		kat_seq(out, 32, 0x80, 1);
		kat_check("AES-XTS decrypt", back, out, 32);
	}

	{
		const struct ccmode_gcm *e = f->ccaes_gcm_encrypt, *d = f->ccaes_gcm_decrypt;
		assert(e->size <= KAT_CTX && d->size <= KAT_CTX);
		if (f->ccgcm_init_with_iv_fn(e, (ccgcm_ctx *)ctx, 16, key, kat_gcm_iv0) != 0 ||
		    ccgcm_reset(e, (ccgcm_ctx *)ctx) != 0 || f->ccgcm_inc_iv_fn(e, (ccgcm_ctx *)ctx, iv) != 0 ||
		    ccgcm_aad(e, (ccgcm_ctx *)ctx, sizeof(kat_gcm_aad), kat_gcm_aad) != 0 ||
		    ccgcm_update(e, (ccgcm_ctx *)ctx, 37, pt, out) != 0 || ccgcm_finalize(e, (ccgcm_ctx *)ctx, 16, tag) != 0) {
			panic("builtin_crypto: AES-GCM known-answer test failed");
		}
		kat_check("AES-GCM IV", iv, kat_gcm_iv1, 12);
		kat_check("AES-GCM", out, kat_gcm_ct, sizeof(kat_gcm_ct));
		kat_check("AES-GCM tag", tag, kat_gcm_tag, 16);
		memcpy(tag, kat_gcm_tag, 16);
		if (ccgcm_init(d, (ccgcm_ctx *)ctx, 16, key) != 0 || ccgcm_set_iv(d, (ccgcm_ctx *)ctx, 12, kat_gcm_iv1) != 0 ||
		    ccgcm_aad(d, (ccgcm_ctx *)ctx, sizeof(kat_gcm_aad), kat_gcm_aad) != 0 ||
		    ccgcm_update(d, (ccgcm_ctx *)ctx, 37, out, back) != 0 || ccgcm_finalize(d, (ccgcm_ctx *)ctx, 16, tag) != 0) {
			panic("builtin_crypto: AES-GCM decryption known-answer test failed");
		}
		kat_check("AES-GCM decrypt", back, pt, 37);
	}

	{
		const struct ccchacha20poly1305_fns *c = f->ccchacha20poly1305_fns;
		const struct ccchacha20poly1305_info *info = c->info();
		ccchacha20poly1305_ctx cctx;
		if (c->init(info, &cctx, key) != 0 || c->setnonce(info, &cctx, kat_chacha_nonce) != 0 ||
		    c->aad(info, &cctx, sizeof(kat_gcm_aad), kat_gcm_aad) != 0 || c->encrypt(info, &cctx, 37, pt, out) != 0 ||
		    c->finalize(info, &cctx, tag) != 0) {
			panic("builtin_crypto: ChaCha20-Poly1305 known-answer test failed");
		}
		kat_check("ChaCha20-Poly1305", out, kat_chacha_ct, sizeof(kat_chacha_ct));
		kat_check("ChaCha20-Poly1305 tag", tag, kat_chacha_tag, 16);
		if (c->reset(info, &cctx) != 0 || c->setnonce(info, &cctx, kat_chacha_nonce) != 0 ||
		    c->aad(info, &cctx, sizeof(kat_gcm_aad), kat_gcm_aad) != 0 || c->decrypt(info, &cctx, 37, out, back) != 0 ||
		    c->verify(info, &cctx, kat_chacha_tag) != 0) {
			panic("builtin_crypto: ChaCha20-Poly1305 decryption known-answer test failed");
		}
		kat_check("ChaCha20-Poly1305 decrypt", back, pt, 37);
		cc_clear(sizeof(cctx), &cctx);
	}

	{
		const struct ccmode_ecb *e = f->ccdes_ecb_encrypt, *d = f->ccdes_ecb_decrypt;
		kat_seq(back, 8, 0x50, 1);
		e->init(e, (ccecb_ctx *)ctx, 8, kat_des_key);
		e->ecb((ccecb_ctx *)ctx, 1, back, out);
		kat_check("DES", out, kat_des_ct, 8);
		d->init(d, (ccecb_ctx *)ctx, 8, kat_des_key);
		d->ecb((ccecb_ctx *)ctx, 1, out, out + 8);
		kat_check("DES decrypt", out + 8, back, 8);

		const struct ccmode_cbc *ce = f->cctdes_cbc_encrypt, *cd = f->cctdes_cbc_decrypt;
		kat_seq(back, 16, 0x60, 1);
		kat_seq(iv, 8, 0x90, 1);
		cccbc_init(ce, (cccbc_ctx *)ctx, 24, kat_des3_key);
		cccbc_set_iv(ce, (cccbc_iv *)ctx2, iv);
		cccbc_update(ce, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 2, back, out);
		kat_check("3DES-CBC", out, kat_des3_ct, 16);
		cccbc_init(cd, (cccbc_ctx *)ctx, 24, kat_des3_key);
		cccbc_set_iv(cd, (cccbc_iv *)ctx2, iv);
		cccbc_update(cd, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 2, out, out + 16);
		kat_check("3DES-CBC decrypt", out + 16, back, 16);

		uint8_t weak[8] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }, odd[8] = { 0 };
		f->ccdes_key_set_odd_parity_fn(odd, 8);
		if (!f->ccdes_key_is_weak_fn(weak, 8) || f->ccdes_key_is_weak_fn((void *)(uintptr_t)kat_des_key, 8) ||
		    memcmp(odd, weak, 8) != 0) {
			panic("builtin_crypto: DES key helpers failed");
		}
	}

	{
		const struct ccmode_cbc *e = f->ccaes_cbc_encrypt, *d = f->ccaes_cbc_decrypt;
		kat_seq(back, 20, 0x21, 1);
		memset(iv, 0, 16);
		cccbc_init(e, (cccbc_ctx *)ctx, 16, key);
		cccbc_set_iv(e, (cccbc_iv *)ctx2, iv);
		f->ccpad_cts3_encrypt_fn(e, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 20, back, out);
		kat_check("CTS3", out, kat_cts_ct, 20);
		cccbc_init(d, (cccbc_ctx *)ctx, 16, key);
		cccbc_set_iv(d, (cccbc_iv *)ctx2, iv);
		f->ccpad_cts3_decrypt_fn(d, (cccbc_ctx *)ctx, (cccbc_iv *)ctx2, 20, out, out + 20);
		kat_check("CTS3 decrypt", out + 20, back, 20);
	}

	{
		static const uint8_t e65537[] = { 0x01, 0x00, 0x01 };
		uint8_t sig[sizeof(kat_rsa_sig)];
		ccrsa_pub_ctx_t pub = (ccrsa_pub_ctx_t)ctx;
		assert(ccrsa_pub_ctx_size(sizeof(kat_rsa_n)) <= KAT_CTX);
		ccrsa_ctx_n(pub) = ccn_nof_size(sizeof(kat_rsa_n));
		builtin_digest(CRYPTO_DIGEST_ALG_SHA256, "abc", 3, out, 32);
		if (f->ccrsa_make_pub_fn(pub, sizeof(e65537), e65537, sizeof(kat_rsa_n), kat_rsa_n) != 0 ||
		    f->ccrsa_verify_pkcs1v15_fn(pub, ccoid_sha256, 32, out, sizeof(kat_rsa_sig), kat_rsa_sig, &valid) != 0 ||
		    !valid) {
			panic("builtin_crypto: RSA known-answer test failed");
		}
		memcpy(sig, kat_rsa_sig, sizeof(sig));
		sig[17] ^= 4;
		if (f->ccrsa_verify_pkcs1v15_fn(pub, ccoid_sha256, 32, out, sizeof(sig), sig, &valid) != 0 || valid) {
			panic("builtin_crypto: RSA accepted a changed signature");
		}
	}
	cc_clear(sizeof(ctx), ctx);
	cc_clear(sizeof(ctx2), ctx2);
}

#if DEVELOPMENT || DEBUG
#define AES_TEST_LEN    (64 * 1024)
#define AES_TEST_PASSES 16

static uint64_t
builtin_crypto_aes_time(const struct ccmode_ecb *m, size_t key_len, const uint8_t *key,
    const uint8_t *in, uint8_t *out)
{
	uint8_t ctx[512] __attribute__((aligned(16)));
	uint64_t start, ns;

	assert(m->size <= sizeof(ctx));
	start = mach_absolute_time();
	m->init(m, (ccecb_ctx *)ctx, key_len, key);
	for (int i = 0; i < AES_TEST_PASSES; i++) {
		m->ecb((ccecb_ctx *)ctx, AES_TEST_LEN / 16, in, out);
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - start, &ns);
	cc_clear(sizeof(ctx), ctx);
	return ns ? ns : 1;
}

static int
builtin_crypto_aes_test(__unused int64_t in, int64_t *out)
{
	uint8_t *buf = kalloc_data(3 * AES_TEST_LEN, Z_WAITOK);
	uint8_t key[32];
	bool ok = true;

	if (buf == NULL) {
		return ENOMEM;
	}
	read_random(key, sizeof(key));
	read_random(buf, AES_TEST_LEN);
	for (size_t key_len = 16; key_len <= 32; key_len += 16) {
		for (int dec = 0; dec < 2; dec++) {
			const struct ccmode_ecb *used = dec ? ccaes_ecb_decrypt_mode() : ccaes_ecb_encrypt_mode();
			const struct ccmode_ecb *soft = dec ? &ccaes_ltc_ecb_decrypt_mode : &ccaes_ltc_ecb_encrypt_mode;
			bool hw = pc_cpu_has_aes();
			uint64_t t_used = builtin_crypto_aes_time(used, key_len, key, buf, buf + AES_TEST_LEN);
			uint64_t t_soft = builtin_crypto_aes_time(soft, key_len, key, buf, buf + 2 * AES_TEST_LEN);
			uint64_t bytes = (uint64_t)AES_TEST_LEN * AES_TEST_PASSES;
			bool same = memcmp(buf + AES_TEST_LEN, buf + 2 * AES_TEST_LEN, AES_TEST_LEN) == 0;

			printf("builtin_crypto: AES-%zu %s: %llu MB/s %s, %llu MB/s bitsliced%s\n", key_len * 8,
			    dec ? "decrypt" : "encrypt", bytes * 1000 / t_used, hw ? "with the instructions" : "in use",
			    bytes * 1000 / t_soft, same ? "" : "; THE OUTPUTS DIFFER");
			ok = ok && same;
		}
	}
	cc_clear(sizeof(key), key);
	kfree_data(buf, 3 * AES_TEST_LEN);
	*out = ok;
	return ok ? 0 : EIO;
}
SYSCTL_TEST_REGISTER(builtin_crypto_aes, builtin_crypto_aes_test);
#endif

__startup_func
static void
builtin_crypto_init(void)
{
	bool hw = pc_cpu_has_aes();
	int use_hw = 1;

	builtin_crypto_self_test();
	builtin_crypto_fill_modes(&builtin_crypto_funcs);
	if (hw) {
		pc_cpu_use_hw(false);
		builtin_crypto_cipher_self_test(&builtin_crypto_funcs);
		pc_cpu_use_hw(true);
	}
	builtin_crypto_cipher_self_test(&builtin_crypto_funcs);
	if (hw && PE_parse_boot_argn("piecrypto_hw", &use_hw, sizeof(use_hw)) && use_hw == 0) {
		pc_cpu_use_hw(false);
		hw = false;
	}
	printf("builtin_crypto: AES %s\n", hw ? "with the ARMv8 instructions" : "bitsliced");

	builtin_crypto_funcs.ccmd5_di = ccmd5_di();
	builtin_crypto_funcs.ccsha1_di = ccsha1_di();
	builtin_crypto_funcs.ccsha256_di = ccsha256_di();
	builtin_crypto_funcs.ccsha384_di = ccsha384_di();
	builtin_crypto_funcs.ccsha512_di = ccsha512_di();
	if (register_crypto_functions(&builtin_crypto_funcs) != 0) {
		panic("builtin_crypto: crypto functions already registered");
	}
	register_and_init_prng((struct cckprng_ctx *)&kprng, &builtin_kprng_funcs);
}
STARTUP(SYSCTL, STARTUP_RANK_LAST, builtin_crypto_init);
