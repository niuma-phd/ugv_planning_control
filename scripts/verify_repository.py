#!/usr/bin/env python3
"""Small dependency-free structural verifier for the UGV MVP repository."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ERRORS: list[str] = []


def require(path: str) -> Path:
    result = ROOT / path
    if not result.exists():
        ERRORS.append(f"missing required path: {path}")
    return result


def check_package_xml() -> None:
    for path in sorted((ROOT / "src").glob("*/package.xml")):
        try:
            tree = ET.parse(path)
        except ET.ParseError as exc:
            ERRORS.append(f"{path.relative_to(ROOT)}: invalid XML: {exc}")
            continue
        package = tree.getroot()
        if package.tag != "package":
            ERRORS.append(f"{path.relative_to(ROOT)}: root element is not package")
        licenses = [node.text or "" for node in package.findall("license")]
        if "Apache-2.0" not in licenses:
            ERRORS.append(
                f"{path.relative_to(ROOT)}: first-party package must declare Apache-2.0"
            )
        text = path.read_text(encoding="utf-8").lower()
        for forbidden in ("nav2_", "autoware", "pcl_ros"):
            if forbidden in text:
                ERRORS.append(
                    f"{path.relative_to(ROOT)}: forbidden MVP dependency {forbidden}"
                )


def check_markdown_links() -> None:
    link_pattern = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    for path in sorted(ROOT.rglob("*.md")):
        text = path.read_text(encoding="utf-8")
        if text.count("```") % 2:
            ERRORS.append(f"{path.relative_to(ROOT)}: unclosed code fence")
        for target in link_pattern.findall(text):
            if (
                "://" in target
                or target.startswith("#")
                or target.startswith("mailto:")
            ):
                continue
            clean = target.split("#", 1)[0]
            if clean and not (path.parent / clean).resolve().exists():
                ERRORS.append(
                    f"{path.relative_to(ROOT)}: broken relative link {target}"
                )


def check_large_or_runtime_files() -> None:
    forbidden_suffixes = {".db3", ".mcap", ".bag", ".pcd"}
    for path in ROOT.rglob("*"):
        if ".git" in path.parts:
            continue
        if path.is_dir() and path.name == ".omx":
            ERRORS.append(f"runtime state must not be committed: {path.relative_to(ROOT)}")
        if path.is_file():
            if path.suffix.lower() in forbidden_suffixes:
                ERRORS.append(f"large data file in repository: {path.relative_to(ROOT)}")
            if path.stat().st_size > 5 * 1024 * 1024:
                ERRORS.append(f"file exceeds 5 MiB: {path.relative_to(ROOT)}")


def check_roadmaps() -> None:
    expected = [
        "workstreams/00_repository_integration/SESSION_ROADMAP.md",
        "workstreams/01_localization/SESSION_ROADMAP.md",
        "workstreams/02_subject2_control/SESSION_ROADMAP.md",
        "workstreams/03_subject1_perception/SESSION_ROADMAP.md",
        "workstreams/04_subject1_avoidance/SESSION_ROADMAP.md",
        "workstreams/05_gps_lio_recovery/SESSION_ROADMAP.md",
        "workstreams/06_rdk_test_tuning/SESSION_ROADMAP.md",
    ]
    for path in expected:
        require(path)


def main() -> int:
    for path in (
        "AGENTS.md",
        "README.md",
        "docs/ARCHITECTURE.md",
        "docs/INTERFACES.md",
        "docs/ROADMAP.md",
        "docs/KNOWN_GAPS.md",
    ):
        require(path)

    check_package_xml()
    check_markdown_links()
    check_large_or_runtime_files()
    check_roadmaps()

    print(f"PACKAGE_XML_OK {len(list((ROOT / 'src').glob('*/package.xml')))}")
    print(f"MARKDOWN_OK {len(list(ROOT.rglob('*.md')))}")
    if ERRORS:
        for error in ERRORS:
            print(f"ERROR {error}")
        print(f"ERROR_COUNT {len(ERRORS)}")
        return 1
    print("ERROR_COUNT 0")
    return 0


if __name__ == "__main__":
    sys.exit(main())

