#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import os
import re
import sys

import matplotlib.pyplot as plt


RE_CYCLE = re.compile(
    r"Ciclo Completo:\s*Inicio:\s*([0-9]+(?:\.[0-9]+)?),\s*fin:\s*([0-9]+(?:\.[0-9]+)?),\s*duracion:\s*([0-9]+(?:\.[0-9]+)?)"
)
RE_PARTS = re.compile(
    r"Duraciones ciclo \[s\] \| update_wheel_speed: ([0-9.]+) \| get_pwm_output_pid: ([0-9.]+) \| "
    r"send_pwm_cmd: ([0-9.]+) \| pub_wheel_speed: ([0-9.]+) \| pub_encoder: ([0-9.]+) \| "
    r"pub_pwm: ([0-9.]+) \| update_odom: ([0-9.]+)"
)

NAMES = [
    "update_wheel_speed",
    "get_pwm_output_pid",
    "send_pwm_cmd",
    "pub_wheel_speed",
    "pub_encoder",
    "pub_pwm",
    "update_odom",
]


def parse_log(path):
    starts = []
    total = []
    parts = []

    with open(path, "r") as f:
        for line in f:
            m = RE_CYCLE.search(line)
            if m:
                starts.append(float(m.group(1)))
                total.append(float(m.group(3)))
                continue
            m = RE_PARTS.search(line)
            if m:
                parts.append([float(x) for x in m.groups()])

    n = min(len(starts), len(parts), len(total))
    return starts[:n], total[:n], parts[:n]


def main():
    parser = argparse.ArgumentParser(
        description="Grafica duracion vs tiempo para todas las funciones del ciclo."
    )
    parser.add_argument("--input", default="logs_tiempo_completo.txt")
    parser.add_argument("--output", default="cycle_breakdown_vs_time.png")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print("Archivo no encontrado: %s" % args.input)
        return 1

    starts, total, parts = parse_log(args.input)
    if not starts:
        print("No se encontraron datos parseables en %s" % args.input)
        return 1

    t0 = starts[0]
    t_rel = [t - t0 for t in starts]

    plt.figure(figsize=(13, 6))
    plt.plot(t_rel[0:20], [x * 1000.0 for x in total][0:20], linewidth=1.2, label="ciclo_total")

    cols = list(zip(*parts))
    for name, col in zip(NAMES, cols):
        plt.plot(t_rel, [x * 1000.0 for x in col], linewidth=0.9, label=name)

    plt.axhline(1000.0 / 60.0, linestyle="--", linewidth=1.0, label="objetivo_60Hz (16.67 ms)")
    plt.xlabel("Tiempo desde inicio [s]")
    plt.ylabel("Duracion [ms]")
    plt.title("Duracion vs tiempo: ciclo completo y funciones internas")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right", ncol=2, fontsize=8)
    plt.tight_layout()
    plt.savefig(args.output, dpi=160)

    print("Muestras: %d" % len(t_rel))
    print("Grafico guardado en: %s" % args.output)

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())

