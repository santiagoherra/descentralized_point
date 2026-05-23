#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse
import os
import re
import sys

import matplotlib.pyplot as plt


LINE_RE = re.compile(
    r"Ciclo Completo:\s*Inicio:\s*([0-9]+(?:\.[0-9]+)?),\s*fin:\s*([0-9]+(?:\.[0-9]+)?),\s*duracion:\s*([0-9]+(?:\.[0-9]+)?)"
)


def parse_cycles(path):
    starts = []
    ends = []
    durations = []

    with open(path, "r") as f:
        for line in f:
            m = LINE_RE.search(line)
            if not m:
                continue
            starts.append(float(m.group(1)))
            ends.append(float(m.group(2)))
            durations.append(float(m.group(3)))

    return starts, ends, durations


def main():
    parser = argparse.ArgumentParser(
        description="Grafica duracion del ciclo completo vs tiempo desde logs ROS."
    )
    parser.add_argument(
        "--input",
        default="logs_tiempo.txt",
        help="Ruta al archivo de logs (default: logs_tiempo.txt)",
    )
    parser.add_argument(
        "--output",
        default="cycle_duration_vs_time.png",
        help="Ruta de salida para la figura PNG (default: cycle_duration_vs_time.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Mostrar grafico en pantalla ademas de guardarlo",
    )
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print("Archivo no encontrado: %s" % args.input)
        return 1

    starts, ends, durations = parse_cycles(args.input)
    if not durations:
        print("No se encontraron lineas de 'Ciclo Completo' en %s" % args.input)
        return 1

    t0 = starts[0]
    t_rel = [t - t0 for t in starts]
    durations_ms = [d * 1000.0 for d in durations]

    avg_ms = sum(durations_ms) / float(len(durations_ms))
    min_ms = min(durations_ms)
    max_ms = max(durations_ms)

    plt.figure(figsize=(11, 5))
    plt.plot(t_rel[0:50], durations_ms[0:50], linewidth=1.1, label="Duracion ciclo")
    plt.axhline(avg_ms, linestyle="--", linewidth=1.0, label="Promedio: %.2f ms" % avg_ms)
    plt.axhline(1000.0 / 60.0, linestyle=":", linewidth=1.0, label="Objetivo 60 Hz: 16.67 ms")
    plt.title("Duracion del ciclo completo vs tiempo")
    plt.xlabel("Tiempo desde inicio [s]")
    plt.ylabel("Duracion del ciclo [ms]")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)

    print("Muestras: %d" % len(durations_ms))
    print("Min: %.3f ms | Promedio: %.3f ms | Max: %.3f ms" % (min_ms, avg_ms, max_ms))
    print("Grafico guardado en: %s" % args.output)

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())

