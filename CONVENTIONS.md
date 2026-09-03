# 코드 컨벤션 (Code Conventions)

> 이 문서는 Team2 MORAI 프로젝트의 모든 패키지에 공통으로 적용되는 코딩·협업 규칙이다.

> 새 코드를 작성하거나 PR을 올릴 때 이 문서를 기준으로 한다.



## 1. 명명 규칙 (Naming)

### 원칙

- 대소문자를 엄격하게 구분한다.
- 변수는 명사로 표기한다. (예: `count`, `number`)
- 함수는 동사 또는 동사구로 표기한다. (예: `find_camera()`, `get_foo()`)
  - 함수는 snake_case로 쓴다. ROS API(`create_subscription`, `get_parameter`, `advertise`)가 전부 snake_case이므로 표기를 통일한다.
- 상수는 대문자 언더스코어 방식을 쓴다.
  - (예: `PI`, `MAX_VERTEX`)
- 축약x, 의미를 알기 쉽게 풀어쓴다.
  - `msg` 대신 `message`

### C++

| 대상 | 규칙 | 예시 |
| --- | --- | --- |
| 클래스·구조체 | PascalCase | `UdpRosBridge` |
| 함수·메서드 | snake_case (동사로 시작) | `receive_packet()` |
| 지역변수·매개변수 | snake_case | `packet_size` |
| 멤버변수 | snake_case_ (끝에 `_`) | `socket_fd_` |
| 전역변수 | g_snake_case | `g_shutdown_requested` |
| 상수 | kPascalCase | `kBufferSize` |
| 매크로 | UPPER_SNAKE_CASE | `CHECK_SOCKET_ERROR` |
| 네임스페이스 | snake_case | `rescue_robot` |
| 파일명 | snake_case | `udp_ros_bridge.cpp` |
| ROS 토픽·파라미터 | snake_case | `/rescue_request` |
| 메시지(.msg) 필드 | snake_case (언어 무관) | `segment_point_index` |


**물리량에는 단위를 접미사로 붙인다.**
  - `distance_m`, `timeout_ms`, `velocity_mps` — 변수·매개변수·msg 필드 공통.

### Python

| 대상 | C++ | Python |
| --- | --- | --- |
| 클래스·구조체 | `PascalCase` | `PascalCase` (동일) |
| 함수·메서드 | `snake_case` | `snake_case` (동일) |
| 변수·매개변수 | `snake_case` | `snake_case` (동일) |
| 멤버변수 | `snake_case_` | `snake_case` (뒤 `_` 없음) |
| 상수 | `kBufferSize` | `UPPER_SNAKE_CASE` |
| ROS 토픽·파라미터 | `snake_case` | `snake_case` (동일) |

---
</br>


## 2. 포맷 규칙 (Formatting)

- 한 줄 최대 100자 정도.
- 중괄호는 선언과 같은 줄에 둔다. (C++)
  ```cpp
  if (connected) {
    receive_packet();
  }
  ```
- 포인터·참조는 타입에 붙인다.
  - `int* value`, `const Message& message`
- 모든 `if` / `for` / `while`에 중괄호를 쓴다. 
- 헤더(`.hpp`)에는 선언, `.cpp`에는 구현.

---
</br>


## 3. 주석 컨벤션 (Comments)

- **파일 헤더 주석:** 파일 맨 위에 담긴 클래스, 클래스 간 소통 방식, 코드의 역할을 간략히 적는다. ROS 노드면 **sub/pub 토픽(인터페이스)도 함께 명시**한다.
  ```cpp
  /*
  udp_ros_bridge.cpp
  - 역할: MORAI Sim의 UDP 센서 패킷을 ROS 토픽으로 변환하고,
          ROS 제어 명령을 UDP로 MORAI에 전달한다.
  - 주요 클래스: UdpRosBridge
  인터페이스
  - pub /ego_odometry: nav_msgs/Odometry
  - sub /ctrl_cmd: <제어 명령 메시지>
  */
  ```
- **함수(Function) 주석:** 함수 위에 `함수이름 / 기능 / 인자 / 반환값`을 적는다.
  ```cpp
  // 함수이름: receive_packet
  // 기능: UDP 소켓에서 한 패킷을 읽어 파싱한다.
  // 인자: 없음
  // 반환값: 성공하면 true
  bool receive_packet();
  ```
- **블록(Block) 주석:** 위와 같이 블록에 대한 간략한 요약을 한다(함수가 아니어도 구조체나 클래스일 경우). 라인 주석으로 한다 ( // )
- **문장(Sentence) 주석:** 한 문장에 대한 간략한 설명. 블록에 대한 간략한 요약을 적는다. 이 역시 라인 주석으로 한다 ( // )

---
</br>

## 4. 커밋 / README 규칙

- **코드의 함수·구조체·클래스에 맞춰 다이어그램을 만든다.**
  - **시퀀스 다이어그램**과 **클래스 다이어그램**을 Mermaid로 작성해 각 패키지 `README.md`에 넣는다.
- **구조나 인터페이스를 바꾸면, 같은 PR 안에서 README도 함께 고친다.**

</br>

### 머메이드 다이어그램 예시

</br>

![시퀀스 다이어그램 1](docs/convention_example_img/line_pipeline_sequence.png)

![시퀀스 다이어그램 2](docs/convention_example_img/traffic_light_sequence.png)

![클래스 다이어그램 1](docs/convention_example_img/main_class_diagram.png)

![클래스 다이어그램 2](docs/convention_example_img/tool_class_diagram.png)


### Git 작업 흐름

main에 직접 커밋하지 않고 브랜치 → PR → 병합으로 진행한다. (`README.md` 참고)

```bash

git switch main
git pull --ff-only origin main
git switch -c feature/작업이름

# 작업 후
git add .
git commit -m "작업 내용"
git push -u origin feature/작업이름
# GitHub에서 Pull Request 생성 → Squash and merge

```


