# HD Map RViz 표시 검증

- 기존 hd_map_pkg MGeo importer와 좌표 변환 함수를 재사용한다.
- 중앙 map_projection.yaml의 UTM 원점을 사용하며 지도 원점을 차량 추정에 맞춰 이동하지 않는다.
- 기본 localization_visualization.launch에 HD Map 표시를 연결했다.
- 지도 2,299개 경계선과 1,317개 중심선, 마커 정점 33,124개를 확인했다.
- 별도 ROS master에서 bringup launch(RViz 비활성 옵션)의 map 마커 및 차량 표시 토픽 수신을 확인했다.
- 지도는 표시 전용 평면 투영이다. 원본 높이 및 차량 추정값은 변경하지 않는다.
- 좌표 원점 변환, CRS/NaN 거부, 마커 평면 투영 테스트를 추가했다.
- 실제 GPS/IMU 수신 중 MORAI 지도 정합과 데스크톱 RViz 렌더링은 별도 확인 대상이다.
