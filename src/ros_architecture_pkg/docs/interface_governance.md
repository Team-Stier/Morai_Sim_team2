# 공개 인터페이스 변경 절차

## 단일 원본

`../config/interface_contract.yaml`이 공개 ROS 인터페이스의 기계 판독 가능한
단일 원본이다. 패키지 README의 node/topic/type 표와 `docs/interface_io.*`는
개발자가 자기 경계를 바로 확인하도록 제공하는 읽기용 투영이며 별도의 원본이
아니다.

## 변경 순서

1. 문제와 필요한 논리 데이터를 설명한다.
2. 기존 인터페이스 재사용 가능성을 확인한다.
3. owner, producer, consumer와 dependency 방향을 정한다.
4. type, unit, frame, timestamp source, rate, queue, timeout과 invalid policy를 결정한다.
5. 중앙 계약과 필요한 공유 타입을 변경한다.
6. producer와 모든 consumer를 같은 변경 단위에서 수정한다.
7. 단위 테스트, 계약 테스트, launch/build와 실제 callback 흐름을 검증한다.
8. `scripts/generate_interface_diagrams.py`로 모든 Mermaid를 다시 생성하고
   `--check`와 README projection 검사를 통과시킨다.

## 금지 사항

- 임시 공개 topic/node/frame 이름을 코드에 먼저 넣고 나중에 문서화하기
- 같은 의미의 메시지를 패키지마다 새로 정의하기
- producer만 변경하고 consumer·launch·config를 남겨두기
- frame 또는 단위가 불명확한 좌표를 world model에 넣기
- timeout과 invalid policy 없는 입력을 안전 경로에 사용하기
- 공개 node/topic을 통합 launch에서 remap하거나 anonymous name으로 실행하기
- 다른 패키지의 `/molit/internal/...` topic을 직접 구독하기
