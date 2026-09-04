# Repository instructions for agents

## Implementation discipline

- For inquiry tasks, report findings instead of changing files unless edits are requested.
- Before changing behavior, read the relevant code, tests, and local guidance. State material assumptions; ask only when ambiguity makes the change risky.
- Keep changes minimal and focused on the request. Do not add speculative features, abstractions, configurability, or unrelated refactors.
- Preserve existing external interfaces unless the request requires changing them. If APIs, CLI flags, config, environment variables, ROS interfaces, Docker settings, or model schemas change, call it out explicitly.
- Match existing style and patterns, even if another approach would also work.
- Do not revert or overwrite user changes unless explicitly requested. Clean up only unused code introduced by your own changes.
- Ask for confirmation before destructive operations, hard-to-reverse changes, or material scope expansion.
- For multi-step tasks, use a brief plan tied to verifiable outcomes.
- Before finishing, run the most focused relevant validation and report any blocker clearly.

## Conventional commits

All commit messages MUST follow the Conventional Commits specification.
Use only one of the following types, intentionally aligned with the repository's `conventional-pre-commit` validation:

`build`, `chore`, `ci`, `docs`, `feat`, `fix`, `perf`, `refactor`, `revert`, `style`, or `test`.

- Use the format `type: description`. Use a lowercase type.
- Write the description as a concise imperative statement without a trailing period.
- Mark breaking changes with `!` before the colon or with a `BREAKING CHANGE:` footer.
- If `!` is used, the description should describe the breaking change; a `BREAKING CHANGE:` footer may still be added for extra detail.
- Add a concise body after a blank line explaining what changed and why it matters.
- Do not repeat the diff or write an exhaustive changelog.
