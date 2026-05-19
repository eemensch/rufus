#!/usr/bin/env python3
"""Generate per-instance gz model SDFs and a world SDF from an
agent manifest.

Reads a YAML manifest with one entry per agent (id, type,
instance, role, spawn pose). For each agent, instantiates the
matching model template (currently `r1_rover.sdf.in` for rovers)
into a per-instance model directory under the output tree, with
`<model name>` and `<fdm_port_in>` substituted. Then assembles
the world SDF by injecting one `<include>` block per agent into
the world template, plus any decorative `scenery` entries from
the manifest.

Invoked at build time by `rufus_sim_worlds`'s CMakeLists.txt; the
output tree is then installed via `install(DIRECTORY ...)`. The
manifest is the single source of truth for the multi-agent
scenario; hand-editing the generated SDFs is not supported.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml


# Per-type FDM port base. ardurover/arducopter/arduplane SITL all
# use 9002 by default; `-I N` shifts it by 10·N. The gz plugin's
# `<fdm_port_in>` must match.
FDM_PORT_BASE = 9002
FDM_PORT_STRIDE = 10

# Agent type -> template-file prefix. The corresponding files
# `templates/<prefix>.sdf.in` and `templates/<prefix>.config.in`
# must exist; the generated per-instance dir is named
# `<prefix>_inst<instance>`.
TYPE_TO_TEMPLATE = {
    'rover': 'r1_rover',
    'quad': 'iris',
    'plane': 'zephyr',
}


def _render(template_text: str, subs: dict) -> str:
    out = template_text
    for key, value in subs.items():
        out = out.replace(f'__{key}__', str(value))
    return out


def _agent_fdm_port(agent: dict) -> int:
    return FDM_PORT_BASE + FDM_PORT_STRIDE * int(agent['instance'])


def _agent_model_name(agent: dict) -> str:
    prefix = TYPE_TO_TEMPLATE[agent['type']]
    return f"{prefix}_inst{int(agent['instance'])}"


def _agent_include_block(agent: dict) -> str:
    name = agent['id']
    model_uri = f"model://{_agent_model_name(agent)}"
    xyz = agent['spawn']['xyz']
    rpy = agent['spawn']['rpy_degrees']
    pose = (
        f"{xyz[0]} {xyz[1]} {xyz[2]} "
        f"{rpy[0]} {rpy[1]} {rpy[2]}"
    )
    return (
        f"    <include>\n"
        f"      <uri>{model_uri}</uri>\n"
        f"      <name>{name}</name>\n"
        f"      <pose degrees=\"true\">{pose}</pose>\n"
        f"    </include>"
    )


def _scenery_include_block(entry: dict) -> str:
    pose = entry.get('pose_degrees', '0 0 0 0 0 0')
    return (
        f"    <include>\n"
        f"      <uri>{entry['uri']}</uri>\n"
        f"      <pose degrees=\"true\">{pose}</pose>\n"
        f"    </include>"
    )


def _emit_agent_model(agent: dict, templates_dir: Path,
                      models_out: Path) -> None:
    template_prefix = TYPE_TO_TEMPLATE[agent['type']]
    model_name = _agent_model_name(agent)
    fdm_port = _agent_fdm_port(agent)

    sdf_in = (templates_dir / f'{template_prefix}.sdf.in').read_text()
    cfg_in = (templates_dir / f'{template_prefix}.config.in').read_text()
    subs = {'NAME': model_name, 'FDM_PORT_IN': fdm_port}

    out_dir = models_out / model_name
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'model.sdf').write_text(_render(sdf_in, subs))
    (out_dir / 'model.config').write_text(_render(cfg_in, subs))


def _emit_world(manifest: dict, templates_dir: Path,
                worlds_out: Path) -> None:
    world_in = (templates_dir / 'world.sdf.in').read_text()
    blocks = [_scenery_include_block(s)
              for s in manifest.get('scenery') or []]
    blocks += [_agent_include_block(a) for a in manifest['agents']]
    subs = {
        'WORLD_NAME': manifest['world_name'],
        'INCLUDES': '\n\n'.join(blocks),
    }
    worlds_out.mkdir(parents=True, exist_ok=True)
    (worlds_out / f"{manifest['world_name']}.sdf").write_text(
        _render(world_in, subs))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--templates-dir', required=True, type=Path)
    parser.add_argument('--out-dir', required=True, type=Path,
                        help='Output root; receives `models/` and '
                             '`worlds/` subdirs.')
    parser.add_argument('--clean', action='store_true',
                        help='Remove out-dir before generating.')
    args = parser.parse_args()

    manifest = yaml.safe_load(args.manifest.read_text())
    if 'world_name' not in manifest or 'agents' not in manifest:
        print(f'manifest missing world_name/agents: {args.manifest}',
              file=sys.stderr)
        return 1

    if args.clean and args.out_dir.exists():
        shutil.rmtree(args.out_dir)

    models_out = args.out_dir / 'models'
    worlds_out = args.out_dir / 'worlds'
    for agent in manifest['agents']:
        if agent['type'] not in TYPE_TO_TEMPLATE:
            print(f"unsupported agent type: {agent['type']}",
                  file=sys.stderr)
            return 1
        _emit_agent_model(agent, args.templates_dir, models_out)
    _emit_world(manifest, args.templates_dir, worlds_out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
