import argparse
import re
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ParamDoc:
    name: str
    type_name: str
    description: str = ""


@dataclass
class FunctionDoc:
    name: str
    return_type: str
    summary: str
    params: list[ParamDoc] = field(default_factory=list)
    return_description: str = ""


def _normalize_space(value: str) -> str:
    return " ".join(value.replace("\n", " ").replace("\t", " ").split())


def _strip_doc_prefix(raw_line: str) -> str:
    return re.sub(r"^\s*\* ?", "", raw_line.rstrip("\n"))


def _trim_blank_edges(lines: list[str]) -> list[str]:
    start = 0
    end = len(lines)
    while start < end and lines[start].strip() == "":
        start += 1
    while end > start and lines[end - 1].strip() == "":
        end -= 1
    return lines[start:end]


def _lines_to_text(lines: list[str]) -> str:
    return "\n".join(_trim_blank_edges(lines))


def _markdown_cell(text: str) -> str:
    if not text:
        return ""
    safe = text.replace("|", r"\|")
    rendered_lines: list[str] = []
    for line in safe.splitlines():
        leading_spaces = len(line) - len(line.lstrip(" "))
        prefix = "&nbsp;" * leading_spaces
        rendered_lines.append(prefix + line.lstrip(" "))
    return "<br>".join(rendered_lines)


def _split_params(raw_params: str) -> list[str]:
    parts: list[str] = []
    depth = 0
    current = []
    for char in raw_params:
        if char == "(":
            depth += 1
        elif char == ")":
            depth = max(depth - 1, 0)
        if char == "," and depth == 0:
            part = "".join(current).strip()
            if part:
                parts.append(part)
            current = []
            continue
        current.append(char)
    tail = "".join(current).strip()
    if tail:
        parts.append(tail)
    return parts


def extract_header_intro(source: str) -> str:
    """Extract comment text from the top of a header file.

    Captures leading block (`/* ... */`) and line (`// ...`) comments until the
    first non-comment token, then returns combined markdown-friendly text.
    """
    lines = source.splitlines()
    idx = 0
    intro_parts: list[str] = []

    while idx < len(lines) and lines[idx].strip() == "":
        idx += 1

    while idx < len(lines):
        stripped = lines[idx].lstrip()

        if stripped.startswith("/*"):
            block_lines: list[str] = []
            line = lines[idx]

            if "*/" in line:
                start = line.find("/*") + 2
                end = line.find("*/", start)
                block_lines.append(line[start:end])
                idx += 1
            else:
                start = line.find("/*") + 2
                block_lines.append(line[start:])
                idx += 1

                while idx < len(lines):
                    current = lines[idx]
                    if "*/" in current:
                        end = current.find("*/")
                        block_lines.append(current[:end])
                        idx += 1
                        break
                    block_lines.append(current)
                    idx += 1

            cleaned = [_strip_doc_prefix(raw) for raw in block_lines]
            intro_text = _lines_to_text(cleaned)
            if intro_text:
                intro_parts.append(intro_text)

            while idx < len(lines) and lines[idx].strip() == "":
                idx += 1
            continue

        if stripped.startswith("//"):
            line_comment_lines: list[str] = []
            while idx < len(lines):
                current = lines[idx].lstrip()
                if not current.startswith("//"):
                    break
                line_comment_lines.append(re.sub(r"^\s*//\s?", "", lines[idx]))
                idx += 1

            intro_text = _lines_to_text(line_comment_lines)
            if intro_text:
                intro_parts.append(intro_text)

            while idx < len(lines) and lines[idx].strip() == "":
                idx += 1
            continue

        break

    return "\n\n".join(intro_parts)


def _parse_param(param: str) -> ParamDoc | None:
    cleaned = _normalize_space(param)
    if not cleaned or cleaned == "void":
        return None

    name_match = re.search(r"([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?$", cleaned)
    if not name_match:
        return ParamDoc(name=cleaned, type_name="")

    name = name_match.group(1)
    type_name = cleaned[: name_match.start()].rstrip()
    type_name = _normalize_space(type_name)
    return ParamDoc(name=name, type_name=type_name)


def extract_docstrings(source: str) -> list[FunctionDoc]:
    """Extract structured C function docs from headers.

    Args:
        source: Header file source text.

    Returns:
        Parsed function documentation entries.
    """
    results: list[FunctionDoc] = []

    pattern = re.compile(
        r"/\*\*(.*?)\*/"
        r"\s*"
        r"([A-Za-z_][\w\s\*]*?)\s+"
        r"([A-Za-z_]\w*)\s*"
        r"\(([^)]*(?:\([^)]*\)[^)]*)*)\)",
        re.DOTALL,
    )

    for match in pattern.finditer(source):
        raw_doc, return_type, func_name, raw_params = match.groups()

        summary_lines: list[str] = []
        param_descriptions: dict[str, list[str]] = {}
        return_lines: list[str] = []
        active_section: tuple[str, str | None] = ("summary", None)

        for raw_line in raw_doc.splitlines():
            line = _strip_doc_prefix(raw_line)

            param_match = re.match(r"@param\s+([A-Za-z_]\w*)\s*(.*)", line.strip())
            if param_match:
                param_name = param_match.group(1)
                param_descriptions.setdefault(param_name, [])
                initial = param_match.group(2)
                if initial:
                    param_descriptions[param_name].append(initial)
                active_section = ("param", param_name)
                continue

            return_match = re.match(r"@(return|returns)\s*(.*)", line.strip())
            if return_match:
                initial = return_match.group(2)
                if initial:
                    return_lines.append(initial)
                active_section = ("return", None)
                continue

            if line.strip().startswith("@"):
                active_section = ("summary", None)
                continue

            section_kind, section_name = active_section
            if section_kind == "param" and section_name is not None:
                param_descriptions.setdefault(section_name, []).append(line)
            elif section_kind == "return":
                return_lines.append(line)
            else:
                summary_lines.append(line)

        params: list[ParamDoc] = []
        for param in _split_params(raw_params):
            parsed = _parse_param(param)
            if parsed is None:
                continue
            parsed.description = _lines_to_text(param_descriptions.get(parsed.name, []))
            params.append(parsed)

        results.append(
            FunctionDoc(
                name=func_name,
                return_type=_normalize_space(return_type),
                summary=_lines_to_text(summary_lines),
                params=params,
                return_description=_lines_to_text(return_lines),
            )
        )

    return results


def _format_function_markdown(function_doc: FunctionDoc) -> str:
    lines: list[str] = [f"## `{function_doc.name}`", ""]
    if function_doc.summary:
        lines.extend(function_doc.summary.splitlines())
        lines.append("")

    lines.extend(
        [
            "### Parameters",
            "",
            "| Name | Type | Description |",
            "| --- | --- | --- |",
        ]
    )

    if function_doc.params:
        for param in function_doc.params:
            param_type = param.type_name or "-"
            description = _markdown_cell(param.description)
            lines.append(f"| `{param.name}` | `{param_type}` | {description} |")
    else:
        lines.append("| (none) | - | - |")

    return_description = function_doc.return_description
    if not return_description:
        return_description = "None." if function_doc.return_type == "void" else ""

    lines.extend(
        [
            "",
            "### Returns",
            "",
            "| Type | Description |",
            "| --- | --- |",
            f"| `{function_doc.return_type}` | {_markdown_cell(return_description)} |",
            "",
        ]
    )

    return "\n".join(lines)


def build_markdown(
    header_path: Path, functions: list[FunctionDoc], header_intro: str = ""
) -> str:
    """Build markdown content for a header.

    Args:
        header_path: Header path for title generation.
        functions: Parsed function docs.

    Returns:
        Complete markdown document.
    """
    title = header_path.stem.replace("_", " ").title()
    lines = [f"# {title}", ""]

    if header_intro:
        lines.extend(header_intro.splitlines())
        lines.append("")

    if not functions:
        lines.append("No documented functions found.")
        lines.append("")
        return "\n".join(lines)

    for function_doc in functions:
        lines.append(_format_function_markdown(function_doc).rstrip())
        lines.append("")

    return "\n".join(lines)


def generate_docs(include_root: Path, docs_root: Path) -> tuple[int, list[Path]]:
    """Generate markdown docs for all headers under include_root.

    Args:
        include_root: Directory containing header files.
        docs_root: Output docs root where mirrored markdown files are written.

    Returns:
        Count and list of written markdown files.
    """
    written: list[Path] = []
    for header_path in sorted(include_root.rglob("*.h")):
        source = header_path.read_text(encoding="utf-8")
        header_intro = extract_header_intro(source)
        functions = extract_docstrings(source)
        if not functions:
            continue

        relative = header_path.relative_to(include_root)
        md_path = docs_root / relative.with_suffix(".md")
        md_path.parent.mkdir(parents=True, exist_ok=True)
        markdown = build_markdown(
            header_path=header_path,
            functions=functions,
            header_intro=header_intro,
        )
        md_path.write_text(markdown, encoding="utf-8")
        written.append(md_path)

    return len(written), written


def main() -> None:
    """CLI entrypoint for generating C header markdown docs.

    Returns:
        None.
    """
    parser = argparse.ArgumentParser(
        description=(
            "Generate markdown docs for all .h files under src/include and "
            "overwrite mirrored files under docs/API/C-internals."
        )
    )
    parser.add_argument("--include-root", default="src/include", type=str)
    parser.add_argument("--docs-root", default="docs/API/C-internals", type=str)
    args = parser.parse_args()

    include_root = Path(args.include_root)
    docs_root = Path(args.docs_root)

    count, written = generate_docs(include_root=include_root, docs_root=docs_root)
    print(f"Wrote {count} markdown file(s):")
    for path in written:
        print(f" - {path}")


if __name__ == "__main__":
    main()
