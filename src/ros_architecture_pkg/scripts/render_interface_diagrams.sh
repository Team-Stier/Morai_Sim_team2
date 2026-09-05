#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd "${script_dir}/../../.." && pwd)"
mermaid_cli_version="11.16.0"
mermaid_config="${repository_root}/src/ros_architecture_pkg/config/mermaid_renderer.json"
required_font="Noto Sans CJK KR"

if ! command -v fc-match >/dev/null 2>&1; then
  echo "fc-match is required to verify the Korean diagram font." >&2
  exit 1
fi
resolved_font="$(fc-match -f '%{family}' "${required_font}")"
if [[ "${resolved_font}" != *"${required_font}"* ]]; then
  echo "Required Korean diagram font not found: ${required_font}" >&2
  exit 1
fi

mapfile -t mermaid_files < <(
  find "${repository_root}/src" -type f \
    \( -path "*/docs/interface_io.mmd" -o \
       -path "*/ros_architecture_pkg/docs/system_architecture.mmd" -o \
       -path "*/ros_architecture_pkg/docs/system_nominal_flow.mmd" -o \
       -path "*/ros_architecture_pkg/docs/system_health_safety_flow.mmd" \) \
    | sort
)

if [[ "${#mermaid_files[@]}" -eq 0 ]]; then
  echo "No generated interface Mermaid files found." >&2
  exit 1
fi

for mermaid_file in "${mermaid_files[@]}"; do
  output_base="${mermaid_file%.mmd}"
  png_scale="1.5"
  if [[ "${mermaid_file}" == */system_architecture.mmd ]]; then
    png_scale="6"
  elif [[ "${mermaid_file}" == */system_*_flow.mmd ]]; then
    png_scale="3"
  fi

  npx --yes "@mermaid-js/mermaid-cli@${mermaid_cli_version}" \
    -i "${mermaid_file}" -o "${output_base}.svg" -b transparent \
    -c "${mermaid_config}"
  npx --yes "@mermaid-js/mermaid-cli@${mermaid_cli_version}" \
    -i "${mermaid_file}" -o "${output_base}.png" -b white -s "${png_scale}" \
    -c "${mermaid_config}"
done

PYTHONNOUSERSITE=1 /usr/bin/python3 \
  "${repository_root}/src/ros_architecture_pkg/scripts/generate_interface_diagrams.py" \
  --repository-root "${repository_root}" --write-render-manifest

echo "Rendered ${#mermaid_files[@]} interface diagrams with Mermaid CLI ${mermaid_cli_version}."
