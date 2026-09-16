# 로컬 지도 패키지 이식 검증 (2026-09-10)

원본: `origin/feature/hd-map-lanelet2`의 `eb84b3ae5bb7914e56b1b13409ef0ce6b7d0a37b`.
최신 `origin/main`에서 HD Map 패키지와 `.gitmodules`를 가져왔다.
알고리즘·설정·테스트는 원본 그대로이며 현재 중앙 계약의 공개 I/O 문서와 그림을
유지했다. ROS publisher와 TF는 추가하지 않았다.
기존 로컬 PAIK의 `common_msgs_pkg` Noetic 빌드 순서 수정도 유지했다
(`catkin_python_setup()`을 `generate_messages()`보다 먼저 호출).

## 실행 결과

- 패키지 unittest: 39개 통과.
- 중앙 계약 unittest: 40개 중 39개 통과, 호스트 MORAI 프로필 부재 1개 skip.
- `generate_interface_diagrams.py --check`: 통과.
- launch/manifest XML, config YAML 파싱: 통과.
- ROS Noetic `PYTHONNOUSERSITE=1 catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`: 통과.
- `catkin_make run_tests` 및 `catkin_test_results --all build/test_results`:
  보고 합계 177 tests, 0 errors, 0 failures, 1 skipped.
- `hd_map_tool build-all`: 성공, 14개 pass / 6개 warning / 0개 fail.
- 원본 submodule commit/tree 일치, 원본 변경 없음.
- Lanelet2 2,346개 생성. 제공 전역경로 4,430점의 중심선 정합 최대 오차 0.056271 m.

경고는 원본 선언 해시 불일치, 빈 link 참조, 퇴화 경계 5개 생략,
신호 4개의 연결 미해결, 넓은 파생 교차로 영역, 일부 native successor 연결에 관한 것이다.
상세 결과는 로컬 `data/derived/KATRI_validation_report.json`에 있다.
파생 OSM·routing graph·뷰어 등 6개 파일은 `data/derived/`에 생성하며 Git에는 추가하지 않는다.

이번 검증은 오프라인 변환과 빌드에 한정된다. 실제 ROS 지도 발행,
MORAI 런타임 지도 동일성 및 closed-loop 주행은 검증하지 않았다.
