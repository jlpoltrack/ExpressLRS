# PR Review Insights

Findings from the last 50 merged PRs (#3608–#3749): what gets PRs merged smoothly, what reviewers push back on, and the practices that follow.

## The numbers

- **Who opens them:** about 70% come from core maintainers. The rest are mostly bug fixes from outside contributors.
- **Approvals:** every PR got at least 2. Usually one reviewer tests on hardware and another reviews the code closely.
- **Time to merge:** one-line fixes go in within 1–3 days. Outside bug fixes often take 2–6 weeks (one took 3 months). Big features take 2–6 weeks.
- **Labels:** each PR gets one type label (bugfix, enhancement or housekeeping) plus a version milestone (e.g. V4.2). Fixes for the maintenance branch go in as separate backport PRs (#3623, #3626).
- **Large diffs:** PRs showing +10k lines are mostly regenerated web assets, not hand-written code.

## What made PRs go smoothly

1. **A description that explains the root cause.** The best-received PRs (#3723, #3716, #3728, #3639, #3645, #3620) follow Problem → Cause → Fix, with the exact failure scenario.
2. **Proof from real hardware.** Before/after CRSF channel dumps (#3645), logic-analyzer captures (#3681), 6–8 hour soak runs (#3620), and naming the exact receiver, radio and firmware tested.
3. **Unit tests for protocol and parser changes.** #3716 and #3652 were thanked for adding them.
4. **Small, tightly scoped diffs.** Single-purpose fixes merge fastest.
5. **Quick, specific replies to review comments,** e.g. "Done, restored in 77f92aac", answered on the same inline thread.
6. **Linking the issue** with `Fixes #NNNN`.

## What reviewers push back on most

| Theme | Examples |
|---|---|
| Changing more than needed | "Really it would have been simple to just multiply…" (#3745); "All that is needed is this if statement… leave it as it was" (#3652) |
| Unrelated changes | `.vscode/settings.json` (#3724); reordered headers (#3652) |
| Code style | `{}` initialization and one-line declarations, no forced casts (#3642); `constexpr` (#3711); typed enum instead of `uint8_t` return (#3620); no non-static globals (#3620); pull duplicated blocks into a function (#3609); size arrays from the constant (#3644) |
| Reinventing existing helpers | Move `htobe24` into shared code rather than a local copy (#3632) |
| Commented-out code | Delete it; git keeps the history (#3680). Exception: keep intentional debug lines already commented out (#3723) |
| Flash, RAM and timing budgets | 500 B RAM on ESP8285 flagged (#3731); a 50–75 ns addition in the PWM ISR rejected (#3681) |
| Backward compatibility | Users on old Lua scripts (#3608); EdgeTX sensor display (#3734); targets without serial pins in `hardware.json` (#3711) |
| Naming and domain rules | Regulatory domain names are 5–6 characters; 900 MHz channels need 600 kHz spacing (#3724); generic labels like "External", not "u.FL" (#3684) |
| Unclear reasoning or repro | "Why do you think the shift is required?" (#3632); "Which receiver, firmware, PWM channel?" (#3609) |
| Monolithic features | #3711 asked to split GPS code into separate u-blox and NMEA files; the reorg was welcomed |

## Best practices

- **Description:** Problem / Cause / Fix / Testing, link the issue, name the exact hardware and modes tested.
- **Diff:** smallest change that fixes the problem, match surrounding style, no unrelated files or reformatting.
- **Style:** `{}` initialization, `constexpr`, typed enums, class-scoped or static state, no stray casts, reuse existing helpers.
- **Resources:** call out any flash or RAM cost, especially on ESP8285. Leave ISRs and timing-critical paths alone unless the change is measured.
- **Compatibility:** consider old Lua scripts, EdgeTX/Ethos display, every target's `hardware.json`, and ESP32 variants (C3 and S3 behave differently).
- **Tests:** unit tests for parsers and protocol code; logs or scope captures for hardware-only behaviour.
- **Big features:** split into modules or several PRs; open as a draft for early feedback.
- **Fixes for older releases:** separate backport PR against `4.x-maint` with the right version label.
