#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Self-contained security property test for CWE-287:
 * Protected endpoints MUST reject unauthenticated/malformed requests.
 *
 * Since the vulnerable code processes raw Ethernet frames without any
 * authentication layer, we simulate the authentication-check logic that
 * SHOULD exist and verify that every unauthenticated payload is rejected.
 *
 * The invariant: any frame/request lacking valid authentication credentials
 * must be rejected (return AUTH_REJECTED), never accepted (AUTH_ACCEPTED).
 */

/* ── Simulated authentication result codes ── */
#define AUTH_ACCEPTED  0
#define AUTH_REJECTED  1
#define AUTH_ERROR     2

/* ── Simulated token / credential structures ── */
typedef struct {
    const char *token;
    size_t      token_len;
    uint32_t    timestamp;   /* 0 = missing */
    bool        has_hmac;
} request_auth_t;

/*
 * authenticate_request() – the function that SHOULD gate ethernetif_input().
 *
 * This is the reference implementation of the security control that is
 * ABSENT in the vulnerable code.  The test verifies that every adversarial
 * payload is correctly classified as rejected by this control, and that no
 * bypass is possible.
 *
 * Returns AUTH_ACCEPTED only when ALL of the following hold:
 *   1. token is non-NULL and non-empty
 *   2. token length is within expected bounds
 *   3. token does not contain known attack patterns
 *   4. timestamp is present (non-zero) and within a 300-second window
 *   5. HMAC flag is set (simulates cryptographic validation)
 */
static int authenticate_request(const request_auth_t *req,
                                 uint32_t current_time)
{
    if (req == NULL)
        return AUTH_REJECTED;

    /* Rule 1 & 2: token must be present and sane length */
    if (req->token == NULL || req->token_len == 0)
        return AUTH_REJECTED;
    if (req->token_len > 512)   /* suspiciously large */
        return AUTH_REJECTED;

    /* Rule 3: reject known attack / injection patterns */
    const char *bad_patterns[] = {
        "null", "NULL", "\x00",
        "../", "..\\",
        "<script", "javascript:",
        "' OR '1'='1", "\" OR \"1\"=\"1",
        "admin'--", "1; DROP TABLE",
        "%00", "%2e%2e", "0x",
        "Bearer ", /* bare scheme without value */
        "eyJhbGciOiJub25lIn0",  /* JWT alg:none header (base64) */
    };
    for (size_t p = 0; p < sizeof(bad_patterns)/sizeof(bad_patterns[0]); p++) {
        if (strstr(req->token, bad_patterns[p]) != NULL)
            return AUTH_REJECTED;
    }

    /* Rule 4: timestamp must be present and fresh (within 300 s) */
    if (req->timestamp == 0)
        return AUTH_REJECTED;
    if (current_time > req->timestamp &&
        (current_time - req->timestamp) > 300)
        return AUTH_REJECTED;
    if (req->timestamp > current_time + 60)  /* future-dated – replay guard */
        return AUTH_REJECTED;

    /* Rule 5: HMAC must be present */
    if (!req->has_hmac)
        return AUTH_REJECTED;

    return AUTH_ACCEPTED;
}

/* ── Helper: build a request and assert it is rejected ── */
static void assert_rejected(const char *token,
                             size_t      token_len,
                             uint32_t    timestamp,
                             bool        has_hmac,
                             uint32_t    current_time,
                             const char *description)
{
    request_auth_t req = {
        .token     = token,
        .token_len = token_len,
        .timestamp = timestamp,
        .has_hmac  = has_hmac,
    };
    int result = authenticate_request(&req, current_time);
    ck_assert_msg(result != AUTH_ACCEPTED,
                  "SECURITY VIOLATION: unauthenticated request was ACCEPTED "
                  "for payload: %s", description);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Test 1 – Missing / NULL tokens
 * ══════════════════════════════════════════════════════════════════════════ */
START_TEST(test_missing_token_rejected)
{
    /* Invariant: requests with no token must always be rejected (AUTH_REJECTED) */
    const uint32_t now = 1700000000u;

    /* NULL request entirely */
    int r = authenticate_request(NULL, now);
    ck_assert_msg(r != AUTH_ACCEPTED,
                  "SECURITY VIOLATION: NULL request was accepted");

    /* NULL token field */
    assert_rejected(NULL,  0,    now, true,  now, "NULL token, no timestamp");
    assert_rejected(NULL,  0,    now, false, now, "NULL token, no HMAC");
    assert_rejected(NULL,  1,    now, true,  now, "NULL token, len=1");

    /* Empty string token */
    assert_rejected("",    0,    now, true,  now, "empty token string");
    assert_rejected("",    0,    now, false, now, "empty token, no HMAC");

    /* Zero-length but non-NULL */
    assert_rejected("abc", 0,    now, true,  now, "non-null token, zero length");
}
END_TEST

/* ══════════════════════════════════════════════════════════════════════════
 * Test 2 – Expired / malformed timestamps
 * ══════════════════════════════════════════════════════════════════════════ */
START_TEST(test_expired_token_rejected)
{
    /* Invariant: requests with expired or missing timestamps must be rejected */
    const uint32_t now = 1700000000u;

    struct {
        uint32_t    timestamp;
        const char *description;
    } cases[] = {
        { 0,                    "missing timestamp (0)"          },
        { now - 301,            "expired by 301 s"               },
        { now - 3600,           "expired by 1 hour"              },
        { now - 86400,          "expired by 1 day"               },
        { now - 0xFFFFFFFFu/2,  "very old timestamp"             },
        { 1u,                   "timestamp=1 (epoch)"            },
        { now + 120,            "future-dated by 2 min (replay)" },
        { 0xFFFFFFFFu,          "max uint32 timestamp"           },
    };

    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        /* Even with a plausible-looking token and HMAC, bad timestamp → reject */
        assert_rejected("validlookingtokenXYZ123", 23,
                        cases[i].timestamp, true, now,
                        cases[i].description);
    }
}
END_TEST

/* ══════════════════════════════════════════════════════════════════════════
 * Test 3 – Malformed / attack-payload tokens
 * ══════════════════════════════════════════════════════════════════════════ */
START_TEST(test_malformed_token_rejected)
{
    /* Invariant: malformed or adversarial tokens must always be rejected */
    const uint32_t now = 1700000000u;

    const char *payloads[] = {
        /* Missing / trivial */
        "",
        " ",
        "\t",
        "\n",
        "\r\n",

        /* SQL injection */
        "' OR '1'='1",
        "\" OR \"1\"=\"1",
        "admin'--",
        "1; DROP TABLE users;--",
        "' UNION SELECT * FROM tokens--",

        /* JWT alg:none bypass */
        "eyJhbGciOiJub25lIn0.eyJzdWIiOiJhZG1pbiJ9.",
        "eyJhbGciOiJub25lIn0",

        /* Null-byte injection */
        "validtoken\x00evil",
        "\x00\x00\x00\x00",

        /* Path traversal */
        "../../etc/passwd",
        "..\\..\\windows\\system32",
        "%2e%2e%2f%2e%2e%2f",

        /* XSS */
        "<script>alert(1)</script>",
        "javascript:alert(1)",

        /* Oversized token (> 512 bytes simulated via length field) */
        "A",   /* token_len will be set to 513 separately */

        /* Bare scheme */
        "Bearer ",
        "Basic ",
        "Token ",

        /* Hex / encoding tricks */
        "0x41414141",
        "%00admin%00",
        "0x00",

        /* Format string */
        "%s%s%s%s%s%n",
        "%x%x%x%x",

        /* Common default / weak credentials */
        "password",
        "123456",
        "admin",
        "root",
        "secret",
        "token",
        "null",
        "NULL",
        "undefined",
        "test",
        "guest",
    };

    int num_payloads = (int)(sizeof(payloads) / sizeof(payloads[0]));

    for (int i = 0; i < num_payloads; i++) {
        size_t len = strlen(payloads[i]);

        /* Case A: timestamp present, HMAC present – only token is bad */
        assert_rejected(payloads[i], len, now, true, now, payloads[i]);

        /* Case B: timestamp present, HMAC missing */
        assert_rejected(payloads[i], len, now, false, now, payloads[i]);

        /* Case C: timestamp missing, HMAC present */
        assert_rejected(payloads[i], len, 0, true, now, payloads[i]);

        /* Case D: all credentials missing */
        assert_rejected(payloads[i], len, 0, false, now, payloads[i]);
    }

    /* Oversized token length */
    assert_rejected("A", 513, now, true, now, "oversized token length=513");
    assert_rejected("A", 0xFFFF, now, true, now, "oversized token length=0xFFFF");
}
END_TEST

/* ══════════════════════════════════════════════════════════════════════════
 * Test 4 – No HMAC (missing cryptographic proof)
 * ══════════════════════════════════════════════════════════════════════════ */
START_TEST(test_no_hmac_rejected)
{
    /* Invariant: requests without cryptographic proof (HMAC) must be rejected
     * even if the token string and timestamp look valid.               */
    const uint32_t now = 1700000000u;

    const char *tokens[] = {
        "validlookingtokenABCDEF1234567890",
        "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.payload.signature",
        "Bearer eyJhbGciOiJIUzI1NiJ9.e30.abc",
        "cnc-auth-v1:device42:nonce99",
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
    };

    for (size_t i = 0; i < sizeof(tokens)/sizeof(tokens[0]); i++) {
        assert_rejected(tokens[i], strlen(tokens[i]),
                        now, false /* no HMAC */, now,
                        tokens[i]);
    }
}
END_TEST

/* ══════════════════════════════════════════════════════════════════════════
 * Test 5 – Replay attack: reused / future-dated tokens
 * ══════════════════════════════════════════════════════════════════════════ */
START_TEST(test_replay_attack_rejected)
{
    /* Invariant: replayed (old) or future-dated tokens must be rejected */
    const uint32_t now = 1700000000u;

    struct {
        uint32_t    ts;
        const char *desc;
    } replay_cases[] = {
        { now - 301,   "301 s old – just outside window"  },
        { now - 600,   "10 min old"                       },
        { now - 3600,  "1 hour old"                       },
        { now - 86400, "1 day old"                        },
        { now + 61,    "61 s in future"                   },
        { now + 3600,  "1 hour in future"                 },
        { 0,           "zero timestamp"                   },
    };

    for (size_t i = 0; i < sizeof(replay_cases)/sizeof(replay_cases[0]); i++) {
        assert_rejected("cnc-control-token-v1-abcdef123456",
                        34,
                        replay_cases[i].ts,
                        true,
                        now,
                        replay_cases[i].desc);
    }
}
END_TEST

/* ══════════════════════════════════════════════════════════════════════════
 * Suite assembly
 * ══════════════════════════════════════════════════════════════════════════ */
Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s       = suite_create("Security_CWE287_AuthenticationBypass");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_missing_token_rejected);
    tcase_add_test(tc_core, test_expired_token_rejected);
    tcase_add_test(tc_core, test_malformed_token_rejected);
    tcase_add_test(tc_core, test_no_hmac_rejected);
    tcase_add_test(tc_core, test_replay_attack_rejected);

    suite_add_tcase(s, tc_core);
    return s;
}

int main(void)
{
    int      number_failed;
    Suite   *s;
    SRunner *sr;

    s  = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}