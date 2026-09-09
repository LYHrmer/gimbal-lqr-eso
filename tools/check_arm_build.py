"""Compile the public C library for Cortex-M4F, without a board or HAL project.

This checks target code generation and static archives, not firmware linking,
execution, motor timing, or worst-case execution time on an STM32.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="arm-none-eabi-gcc")
    parser.add_argument("--newlib-include", type=Path,
                        help="Optional headers for an unpacked local toolchain")
    parser.add_argument("--output", type=Path, default=ROOT/"build/arm-check")
    parser.add_argument("--report", type=Path, default=ROOT/"results/arm_compile.json")
    args = parser.parse_args()
    compiler = shutil.which(args.cc)
    if compiler is None:
        parser.error("arm-none-eabi-gcc not found; see docs/stm32_integration.md")
    compiler = Path(compiler).resolve()
    archiver = compiler.with_name("arm-none-eabi-ar")
    if not archiver.is_file():
        parser.error("matching arm-none-eabi-ar not found next to compiler")
    args.output.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c11", "-O2", "-fno-fast-math", "-ffreestanding", "-mthumb",
             "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
             "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra", "-Wpedantic",
             "-Wconversion", "-Wshadow", "-Werror", "-I", str(ROOT/"include"),
             "-I", str(ROOT/"examples/stm32"), "-I", str(ROOT)]
    # Keep GCC's configured cross-tool search. For an installed /usr/bin GCC,
    # -B/usr/bin/ would select the host assembler instead of the ARM assembler.
    if args.newlib_include:
        flags += ["-isystem", str(args.newlib_include.resolve())]
    sources = ("src/yaw_controller.c", "src/gimbal_controller.c", "src/gimbal_coordinates.c",
               "src/dm_mit.c", "src/gm6020.c",
               "examples/stm32/yaw_periodic.c", "examples/stm32/gimbal_periodic.c")
    probe = args.output/"profile_probe.c"
    probe.write_text('#include "variants/dm4310/simulation_config.h"\n'
                     '#include "variants/gm6020/simulation_config.h"\n'
                     '#include "variants/dm4310/pitch_simulation_config.h"\n'
                     '#include "variants/gm6020/pitch_simulation_config.h"\n'
                     'int compile_profile_probe(void) {\n'
                     '    YawConfig d = dm4310_simulation_config();\n'
                     '    YawConfig g = gm6020_simulation_config();\n'
                     '    GimbalConfig dp = dm4310_pitch_simulation_config();\n'
                     '    GimbalConfig gp = gm6020_pitch_simulation_config();\n'
                     '    return yaw_config_valid(&d) && yaw_config_valid(&g) &&\n'
                     '           gimbal_config_valid(&dp) && gimbal_config_valid(&gp);\n}\n')
    objects = {}
    for source in [ROOT/name for name in sources]+[probe]:
        target = args.output/(source.stem+".o")
        subprocess.run([str(compiler), *flags, "-c", str(source), "-o", str(target)], check=True)
        header = target.read_bytes()[:52]
        if header[:4] != b"\x7fELF" or header[4:6] != b"\x01\x01" or \
                int.from_bytes(header[18:20], "little") != 40:
            raise RuntimeError(f"Not a little-endian ELF32 ARM object: {target}")
        objects[source.stem] = target
    for motor, codec in (("dm4310", "dm_mit"), ("gm6020", "gm6020")):
        subprocess.run([str(archiver), "rcs", str(args.output/f"libgimbal_{motor}.a"),
                        str(objects["yaw_controller"]), str(objects["gimbal_controller"]),
                        str(objects["gimbal_coordinates"]),
                        str(objects[codec])], check=True)
    source_names = list(sources)+["include/yaw_controller.h", "include/gimbal_controller.h", "include/gimbal_coordinates.h",
        "include/dm_mit.h", "include/gm6020.h", "examples/stm32/gimbal_periodic.h",
        "examples/stm32/yaw_periodic.h", "variants/dm4310/simulation_config.h",
        "variants/gm6020/simulation_config.h", "variants/dm4310/pitch_simulation_config.h",
        "variants/gm6020/pitch_simulation_config.h", "tools/check_arm_build.py"]
    result = {"passed": True, "target": "Cortex-M4F, Thumb, hard-float, fpv4-sp-d16",
        "scope": "C compilation and static archives only; no board-specific linking/execution or timing claim",
        "compiler": subprocess.check_output([str(compiler), "--version"], text=True).splitlines()[0],
        "compiler_reported_assembler": subprocess.check_output(
            [str(compiler), "-print-prog-name=as"], text=True).strip(),
        "flags": flags, "object_count": len(objects),
        "objects": {name: {"bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} for name, path in objects.items()},
        "source_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_names}}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({key: result[key] for key in ("passed", "target", "compiler", "object_count", "scope")}, indent=2))


if __name__ == "__main__":
    main()
