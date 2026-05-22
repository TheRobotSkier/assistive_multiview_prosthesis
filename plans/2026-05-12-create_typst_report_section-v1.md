# Plan: Create Typst Report Section File

## Objective
Create a `rapport/` directory and convert the Markdown skeleton at `plans/2026-05-12-report_section_grasp_preshaping-v1.md` into a properly formatted Typst `.typ` file. The content remains identical; only the markup syntax changes.

## Implementation Plan

- [ ] Create directory `rapport/` at project root
- [ ] Create `rapport/grasp_preshaping_section.typ` containing the full skeleton converted to Typst syntax:
  - Markdown `##` headings → Typst `==` headings (level 2), `###` → `===` (level 3)
  - Markdown bullet lists (`- `) → Typst bullet lists (`- `)
  - Markdown numbered lists (`1. `) → Typst numbered lists (`+ `)
  - Markdown code fences (```) → Typst raw blocks (```)
  - Markdown tables (`|...|`) → Typst `#table()` or `#grid()` calls
  - Markdown bold (`**text**`) → Typst `*text*`
  - Markdown inline code (`` `code` ``) → Typst `` `code` ``
  - Markdown blockquotes (`>`) → Typst `#quote[]` or `#block[]`
  - The horizontal rules (`---`) → Typst `#line(length: 100%)` or removed (Typst sections already create visual breaks)
  - Add a document header with `#set document(title: ...)` and `#set page(paper: "a4")` for basic styling
  - Add `#set text(font: "New Computer Modern", size: 11pt)` or similar academic font
  - Wrap figure/table placeholders in Typst `#figure[]` blocks with placeholder labels so they render cleanly and can be referenced
  - Keep all content (every bullet, every figure instruction, every table) identical to the Markdown source

## Verification Criteria
- Every section (1–10) from the Markdown skeleton is present in the Typst file
- All 11 figure placeholders and 5 table placeholders are included
- File compiles without errors when run through `typst compile`
- No content is lost or abbreviated relative to the Markdown source
- Typst syntax is correct (headings, lists, tables, raw blocks, bold, inline code)

## Potential Risks and Mitigations
1. **Typst table syntax is verbose**: Markdown tables map to `#table()` calls which are more explicit. Mitigation: use the `#table()` function with `columns:` parameter matching column count; keep it readable.
2. **Typst raw block syntax differs**: Triple-backtick blocks in Markdown use different delimiters in Typst. Mitigation: use Typst's ```` ``` ````-style raw blocks which are actually identical.
3. **Special characters**: Some content contains `<`, `>`, `|` which may need escaping in Typst. Mitigation: wrap in raw blocks or use `\` escaping where needed.
