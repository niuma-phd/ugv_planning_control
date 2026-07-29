#!/usr/bin/env python3
"""Dependency-free structural verifier for the Subject 2 repository."""

from __future__ import annotations

import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ERRORS: list[str] = []
ACTIVE_PACKAGES = {
    "ugv_localization_mvp",
    "ugv_mvp_tools",
    "ugv_subject2_bringup",
    "ugv_subject2_mvp",
}
FIXTURE_MODES = {
    "subject2",
    "subject2_right",
    "subject2_line",
    "subject2_waypoint_file",
}
IGNORED_DIRECTORY_PREFIXES = (
    ".git",
    ".omx",
    ".ruff_cache",
    "build",
    "install",
    "log",
)
IGNORED_DIRECTORY_NAMES = {"__pycache__", ".pytest_cache"}


def ignored(path: Path) -> bool:
    relative = path.relative_to(ROOT)
    for part in relative.parts:
        if part in IGNORED_DIRECTORY_NAMES:
            return True
        if any(
            part == prefix or part.startswith(f"{prefix}_")
            for prefix in IGNORED_DIRECTORY_PREFIXES
        ):
            return True
    return False


def source_files(suffix: str | None = None) -> list[Path]:
    results: list[Path] = []
    for path in ROOT.rglob("*"):
        if ignored(path) or not path.is_file():
            continue
        if suffix is None or path.suffix == suffix:
            results.append(path)
    return sorted(results)


def require(path: str) -> Path:
    result = ROOT / path
    if not result.exists():
        ERRORS.append(f"missing required path: {path}")
    return result


def package_dependencies(relative_path: str) -> set[str]:
    path = require(relative_path)
    if not path.is_file():
        return set()
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        ERRORS.append(f"{relative_path}: invalid XML: {exc}")
        return set()
    return {
        (node.text or "").strip()
        for node in root
        if node.tag.endswith("depend") and (node.text or "").strip()
    }


def check_packages() -> None:
    package_paths = sorted((ROOT / "src").glob("*/package.xml"))
    discovered: set[str] = set()
    for path in package_paths:
        try:
            package = ET.parse(path).getroot()
        except ET.ParseError as exc:
            ERRORS.append(f"{path.relative_to(ROOT)}: invalid XML: {exc}")
            continue
        if package.tag != "package":
            ERRORS.append(f"{path.relative_to(ROOT)}: root element is not package")
            continue
        name = (package.findtext("name") or "").strip()
        discovered.add(name)
        if name != path.parent.name:
            ERRORS.append(
                f"{path.relative_to(ROOT)}: package name {name!r} "
                f"does not match directory {path.parent.name!r}"
            )
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
    if discovered != ACTIVE_PACKAGES:
        ERRORS.append(
            f"active package set {sorted(discovered)}, "
            f"expected {sorted(ACTIVE_PACKAGES)}"
        )

    localization_dependencies = package_dependencies(
        "src/ugv_localization_mvp/package.xml"
    )
    subject2_dependencies = package_dependencies("src/ugv_subject2_mvp/package.xml")
    bringup_dependencies = package_dependencies("src/ugv_subject2_bringup/package.xml")
    tool_dependencies = package_dependencies("src/ugv_mvp_tools/package.xml")
    if (
        "ugv_mvp_tools"
        in localization_dependencies | subject2_dependencies | bringup_dependencies
    ):
        ERRORS.append("production packages must not depend on ugv_mvp_tools")
    required_bringup = {"ugv_localization_mvp", "ugv_subject2_mvp"}
    missing_bringup = sorted(required_bringup - bringup_dependencies)
    if missing_bringup:
        ERRORS.append(
            "src/ugv_subject2_bringup/package.xml: missing production dependencies "
            f"{missing_bringup}"
        )
    if "ugv_subject2_bringup" not in tool_dependencies:
        ERRORS.append("src/ugv_mvp_tools/package.xml: missing ugv_subject2_bringup")


def check_subject2_build_script() -> None:
    path = require("scripts/build_subject2.sh")
    if not path.is_file():
        return
    if os.name != "nt" and path.stat().st_mode & 0o111 == 0:
        ERRORS.append("scripts/build_subject2.sh is not executable")
    text = path.read_text(encoding="utf-8")
    match = re.search(
        r"^PACKAGES=\(\s*(?P<body>.*?)^\)",
        text,
        flags=re.MULTILINE | re.DOTALL,
    )
    if not match:
        ERRORS.append("scripts/build_subject2.sh: missing PACKAGES array")
        return
    actual = re.findall(
        r"^\s*(ugv_[A-Za-z0-9_]+)\s*$",
        match.group("body"),
        flags=re.MULTILINE,
    )
    expected = [
        "ugv_localization_mvp",
        "ugv_subject2_mvp",
        "ugv_subject2_bringup",
    ]
    if actual != expected:
        ERRORS.append(
            f"scripts/build_subject2.sh packages {actual}, expected {expected}"
        )
    for retired in ("scripts/build_subject.sh", "scripts/build_subject1.sh"):
        if (ROOT / retired).exists():
            ERRORS.append(f"retired build entry must not exist: {retired}")


def check_subject1_retired() -> None:
    forbidden_paths = (
        "src/ugv_subject1_perception_mvp",
        "src/ugv_subject1_avoidance_mvp",
        "src/ugv_subject1_bringup",
        "docs/科目一_局部避障使用说明.md",
        "src/ugv_mvp_tools/launch/subject1_fixture.launch.py",
        "src/ugv_mvp_tools/ugv_mvp_tools/pointcloud_fixture_node.py",
        "src/ugv_mvp_tools/ugv_mvp_tools/next_waypoint_fixture_node.py",
        "src/ugv_mvp_tools/ugv_mvp_tools/nominal_cmd_fixture_node.py",
        "src/ugv_mvp_tools/ugv_mvp_tools/static_tf_fixture_node.py",
    )
    for relative in forbidden_paths:
        if (ROOT / relative).exists():
            ERRORS.append(f"retired Subject 1 path must not exist: {relative}")

    scan_roots = (
        ROOT / "src",
        ROOT / "scripts",
        ROOT / ".github/workflows",
        ROOT / "docs",
    )
    patterns = ("ugv_subject1_", "subject1_fixture", "/subject1/")
    for scan_root in scan_roots:
        for path in scan_root.rglob("*"):
            if path == Path(__file__) or ignored(path) or not path.is_file():
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            for pattern in patterns:
                if pattern in text:
                    ERRORS.append(
                        f"{path.relative_to(ROOT)}: retired Subject 1 reference {pattern!r}"
                    )


def check_fixture_surface() -> None:
    for relative in (
        "src/ugv_mvp_tools/launch/subject2_fixture.launch.py",
        "src/ugv_mvp_tools/ugv_mvp_tools/raw_odom_fixture_node.py",
        "scripts/run_fixture_smoke.sh",
        "scripts/verify_fixture_runtime.py",
    ):
        require(relative)

    run_text = (ROOT / "scripts/run_fixture_smoke.sh").read_text(encoding="utf-8")
    runtime_text = (ROOT / "scripts/verify_fixture_runtime.py").read_text(
        encoding="utf-8"
    )
    ci_text = (ROOT / ".github/workflows/ci.yml").read_text(encoding="utf-8")
    surfaces = {
        "run script": set(re.findall(r"^  (subject2[A-Za-z0-9_]*)(?:\)|$)", run_text, re.MULTILINE)),
        "runtime verifier": set(re.findall(r'^\s+"(subject2[A-Za-z0-9_]*)",$', runtime_text, re.MULTILINE)),
        "CI": set(re.findall(r"run_fixture_smoke\.sh (subject2[A-Za-z0-9_]*)", ci_text)),
    }
    for surface, modes in surfaces.items():
        if modes != FIXTURE_MODES:
            ERRORS.append(
                f"{surface} fixture modes {sorted(modes)}, expected {sorted(FIXTURE_MODES)}"
            )

    launch_text = (
        ROOT / "src/ugv_mvp_tools/launch/subject2_fixture.launch.py"
    ).read_text(encoding="utf-8")
    for endpoint in (
        "/livox_odometry_mapped",
        "/localization/odom",
        "/localization/map_odom",
        "/subject2/target_point",
        "/cmd_vel",
    ):
        if endpoint not in launch_text:
            ERRORS.append(f"fixture launch is missing isolated endpoint {endpoint}")


def check_markdown_links() -> None:
    link_pattern = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    for path in source_files(".md"):
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
    for path in source_files():
        if path.suffix.lower() in forbidden_suffixes:
            ERRORS.append(f"large data file in repository: {path.relative_to(ROOT)}")
        if path.stat().st_size > 5 * 1024 * 1024:
            ERRORS.append(f"file exceeds 5 MiB: {path.relative_to(ROOT)}")


def check_yaml_duplicate_keys() -> None:
    key_pattern = re.compile(r"^(\s*)([A-Za-z0-9_.-]+):")
    for path in source_files(".yaml"):
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
        "docs/科目二_上游接入与测试手册.md",
        "docs/实车接口与待办.md",
    ):
        require(path)

    check_packages()
    check_subject2_build_script()
    check_subject1_retired()
    check_fixture_surface()
    check_markdown_links()
    check_large_or_runtime_files()
    check_yaml_duplicate_keys()

    package_count = len(list((ROOT / "src").glob("*/package.xml")))
    markdown_count = len(source_files(".md"))
    print(f"PACKAGE_XML_OK {package_count}")
    print(f"MARKDOWN_OK {markdown_count}")
    if ERRORS:
        for error in ERRORS:
            print(f"ERROR {error}")
        print(f"ERROR_COUNT {len(ERRORS)}")
        return 1
    print("ERROR_COUNT 0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
