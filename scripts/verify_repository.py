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
            ERRORS.append(
                f"runtime state must not be committed: {path.relative_to(ROOT)}"
            )
        if path.is_file():
            if path.suffix.lower() in forbidden_suffixes:
                ERRORS.append(
                    f"large data file in repository: {path.relative_to(ROOT)}"
                )
            if path.stat().st_size > 5 * 1024 * 1024:
                ERRORS.append(f"file exceeds 5 MiB: {path.relative_to(ROOT)}")


def check_build_scripts() -> None:
    for relative_path in (
        "scripts/build_subject.sh",
        "scripts/build_subject1.sh",
        "scripts/build_subject2.sh",
    ):
        path = require(relative_path)
        if path.exists() and not path.is_file():
            ERRORS.append(f"build script is not a file: {relative_path}")
        elif path.exists() and path.stat().st_mode & 0o111 == 0:
            ERRORS.append(f"build script is not executable: {relative_path}")

    script = require("scripts/build_subject.sh")
    if not script.is_file():
        return
    text = script.read_text(encoding="utf-8")
    expected_packages = {
        "subject1": [
            "ugv_localization_mvp",
            "ugv_subject1_perception_mvp",
            "ugv_subject1_avoidance_mvp",
            "ugv_subject1_bringup",
        ],
        "subject2": [
            "ugv_localization_mvp",
            "ugv_subject2_mvp",
            "ugv_subject2_bringup",
        ],
    }
    for subject, expected in expected_packages.items():
        match = re.search(
            rf"^\s*{subject}\)\s*$"
            rf"(?P<body>.*?)"
            rf"^\s*;;\s*$",
            text,
            flags=re.MULTILINE | re.DOTALL,
        )
        if not match:
            ERRORS.append(f"scripts/build_subject.sh: missing {subject} case")
            continue
        actual = re.findall(
            r"^\s+(ugv_[A-Za-z0-9_]+)\s*$",
            match.group("body"),
            flags=re.MULTILINE,
        )
        if actual != expected:
            ERRORS.append(
                "scripts/build_subject.sh: "
                f"{subject} packages {actual}, expected {expected}"
            )


def package_dependencies(relative_path: str) -> set[str]:
    path = require(relative_path)
    if not path.is_file():
        return set()
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return set()
    return {
        (node.text or "").strip()
        for node in root
        if node.tag.endswith("depend") and (node.text or "").strip()
    }


def check_subject_isolation() -> None:
    old_bringup = ROOT / "src/ugv_mvp_bringup"
    if old_bringup.exists():
        ERRORS.append("legacy cross-subject bringup package must not exist")
    for fixture_launch in (
        "src/ugv_mvp_tools/launch/subject1_fixture.launch.py",
        "src/ugv_mvp_tools/launch/subject2_fixture.launch.py",
    ):
        require(fixture_launch)

    rules = {
        "src/ugv_localization_mvp/package.xml": (
            set(),
            {
                "ugv_subject1_perception_mvp",
                "ugv_subject1_avoidance_mvp",
                "ugv_subject1_bringup",
                "ugv_subject2_mvp",
                "ugv_subject2_bringup",
                "ugv_mvp_tools",
            },
        ),
        "src/ugv_subject1_perception_mvp/package.xml": (
            set(),
            {"ugv_subject2_mvp", "ugv_subject2_bringup", "ugv_mvp_tools"},
        ),
        "src/ugv_subject1_avoidance_mvp/package.xml": (
            set(),
            {"ugv_subject2_mvp", "ugv_subject2_bringup", "ugv_mvp_tools"},
        ),
        "src/ugv_subject1_bringup/package.xml": (
            {
                "ugv_localization_mvp",
                "ugv_subject1_perception_mvp",
                "ugv_subject1_avoidance_mvp",
            },
            {"ugv_subject2_mvp", "ugv_subject2_bringup", "ugv_mvp_tools"},
        ),
        "src/ugv_subject2_mvp/package.xml": (
            set(),
            {
                "ugv_subject1_perception_mvp",
                "ugv_subject1_avoidance_mvp",
                "ugv_subject1_bringup",
                "ugv_mvp_tools",
            },
        ),
        "src/ugv_subject2_bringup/package.xml": (
            {"ugv_localization_mvp", "ugv_subject2_mvp"},
            {
                "ugv_subject1_perception_mvp",
                "ugv_subject1_avoidance_mvp",
                "ugv_subject1_bringup",
                "ugv_mvp_tools",
            },
        ),
    }
    for path, (required, forbidden) in rules.items():
        dependencies = package_dependencies(path)
        missing = sorted(required - dependencies)
        crossed = sorted(forbidden & dependencies)
        if missing:
            ERRORS.append(f"{path}: missing production dependencies {missing}")
        if crossed:
            ERRORS.append(f"{path}: cross-subject/test dependencies {crossed}")

    tool_dependencies = package_dependencies("src/ugv_mvp_tools/package.xml")
    expected_tool_dependencies = {
        "ugv_subject1_bringup",
        "ugv_subject2_bringup",
    }
    missing_tool_dependencies = sorted(expected_tool_dependencies - tool_dependencies)
    if missing_tool_dependencies:
        ERRORS.append(
            "src/ugv_mvp_tools/package.xml: missing fixture dependencies "
            f"{missing_tool_dependencies}"
        )


def check_yaml_duplicate_keys() -> None:
    key_pattern = re.compile(r"^(\s*)([A-Za-z0-9_.-]+):")
    for path in sorted(ROOT.rglob("*.yaml")):
        scopes: list[tuple[int, str]] = []
        seen: dict[tuple[str, ...], set[str]] = {}
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            match = key_pattern.match(line)
            if not match:
                continue
            indentation = len(match.group(1).replace("\t", "        "))
            key = match.group(2)
            while scopes and indentation <= scopes[-1][0]:
                scopes.pop()
            scope = tuple(parent_key for _, parent_key in scopes)
            keys = seen.setdefault(scope, set())
            if key in keys:
                ERRORS.append(
                    f"{path.relative_to(ROOT)}:{line_number}: duplicate YAML key "
                    f"{'.'.join((*scope, key))}"
                )
            keys.add(key)
            remainder = line[match.end() :].split("#", 1)[0].strip()
            if not remainder:
                scopes.append((indentation, key))


def main() -> int:
    for path in (
        "AGENTS.md",
        "README.md",
        "docs/科目二_自主导航使用说明.md",
        "docs/科目一_局部避障使用说明.md",
        "docs/实车接口与待办.md",
    ):
        require(path)

    check_package_xml()
    check_markdown_links()
    check_large_or_runtime_files()
    check_build_scripts()
    check_subject_isolation()
    check_yaml_duplicate_keys()

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
