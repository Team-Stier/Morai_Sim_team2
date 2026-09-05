# ROS 인터페이스 다이어그램 생성

노드·토픽·메시지 타입의 원본은
`config/interface_contract.yaml` 하나뿐이다. 전체 상세 graph, README용 두
읽기 view와 각 패키지 I/O Mermaid는 이 계약에서 자동 생성하며 직접 편집하지
않는다.

```bash
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py
```

생성 결과는 다음과 같다.

- `src/ros_architecture_pkg/docs/system_architecture.mmd`: 중앙 계약의 전체 node-topic graph
- `src/ros_architecture_pkg/docs/system_nominal_flow.mmd`: nominal data/control 읽기 view
- `src/ros_architecture_pkg/docs/system_health_safety_flow.mmd`: health/readiness/safety/evaluation 읽기 view
- `src/<package>/docs/interface_io.mmd`: 해당 패키지의 공개 input-node-output graph

계약과 체크인된 Mermaid, SVG, PNG가 같은 생성 세트인지 확인하려면 다음을
실행한다.

```bash
python3 src/ros_architecture_pkg/scripts/generate_interface_diagrams.py --check
```

각 패키지 그림에는 `package_boundaries`에 선언된 공개 I/O만 들어간다.
`/internal/`, `visibility: internal`, `public: false`인 토픽을 패키지 경계에
노출하면 생성 전에 계약 검사가 실패한다. 초록/파랑 실선은 live, 회색/주황
점선은 예약·미구현·비활성 인터페이스다.

사람이 그림만 보고도 흐름을 이해할 수 있도록 패키지 한 줄 역할은
`package_boundaries.<package>.diagram_summary_ko`, node와 topic의 짧은 한국어
설명은 각각 `nodes[].diagram_description_ko`,
`topics[].diagram_description_ko`에서 읽어 이미지 안에 표시한다. 설명을
바꿀 때도 생성된 MMD/SVG/PNG가 아니라 중앙 계약을 먼저 수정한다.

SVG와 PNG는 Mermaid CLI `11.16.0`과
`config/mermaid_renderer.json`의 deterministic ID 설정으로 고정해 렌더링한다.
한국어 글꼴은 `Noto Sans CJK KR`로 고정하며, 렌더 스크립트가 `fc-match`로
설치 여부를 먼저 확인한다.

```bash
bash src/ros_architecture_pkg/scripts/render_interface_diagrams.sh
```

스크립트는 패키지 그림을 1.5배, 두 읽기 view를 3배, 복잡한 전체 상세본을 6배
PNG로 만들고 SVG도 함께 생성한 뒤 `docs/interface_diagram_manifest.json`에 각
MMD/SVG/PNG의 SHA-256과 중앙 계약·renderer config 해시를 기록한다. 따라서
중앙 계약을 바꾸고 이미지를 다시 렌더링하지 않으면 `--check`와 단위 테스트가
실패한다.

두 스크립트는 저장소 전체의 `src/<package>`를 투영하는 **source-tree 전용 개발
도구**이며 catkin install 실행 파일로 설치하지 않는다. Mermaid 원본, 렌더링
이미지, manifest와 패키지 README 표를 같은 변경 단위로 갱신해야 한다.
