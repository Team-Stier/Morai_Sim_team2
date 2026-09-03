# 공개 인터페이스 변경 절차

## 단일 원본

`../config/interface_contract.yaml`이 공개 ROS 인터페이스의 기계 판독 가능한 단일 원본이다. 패키지 README는 책임과 논리 데이터만 설명하고 별도의 이름 목록을 만들지 않는다.

## 변경 순서

1. 문제와 필요한 논리 데이터를 설명한다.
2. 기존 인터페이스 재사용 가능성을 확인한다.
3. owner, producer, consumer와 dependency 방향을 정한다.
4. type, unit, frame, timestamp source, rate, queue, timeout과 invalid policy를 결정한다.
5. 중앙 계약과 필요한 공유 타입을 변경한다.
6. producer와 모든 consumer를 같은 변경 단위에서 수정한다.
7. 단위 테스트, 계약 테스트, launch/build와 실제 callback 흐름을 검증한다.

## 금지 사항

- 임시 공개 topic/node/frame 이름을 코드에 먼저 넣고 나중에 문서화하기
- 같은 의미의 메시지를 패키지마다 새로 정의하기
- producer만 변경하고 consumer·launch·config를 남겨두기
- frame 또는 단위가 불명확한 좌표를 world model에 넣기
- timeout과 invalid policy 없는 입력을 안전 경로에 사용하기
