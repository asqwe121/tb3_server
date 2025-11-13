cmake_minimum_required(VERSION 3.8)
project(tb3_server)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)   # ★ 추가

# 파이썬 패키지(tb3_server 폴더) 설치
ament_python_install_package(${PROJECT_NAME})   # ★ 추가

# 실행 스크립트 설치(이미 있으면 유지)
install(PROGRAMS
  scripts/coordinator
  scripts/qr_reader
  scripts/alarm
  scripts/server
  DESTINATION lib/${PROJECT_NAME}
)

# 런치/설정 설치(있으면)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ament index 등록(패키지 인식 핵심)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()

