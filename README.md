## **💡1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : 물류 자동화를 위한 자동주문 피킹로봇
- 프로젝트 정의 : 물류센터에서 주문받은 상품을 자동으로 선별하고 피킹 하여 지정된 위치로 이송하는 로봇
<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/1547e61d-8c75-444d-8448-e91b52507cbb" />

**1-2. 개발 배경 및 필요성**
- 비대면 서비스의 확산으로 전자상거래 시장이 급성장하면서, 물류센터의 처리 물량이 급격히 증가하였다. 그러나 심각한 인력난과 인건비 상승으로 인해 물류 작업의 효율성과 정확성을 동시에 확보하기 어려운 상황이 지속되고 있다.
기존의 물류 자동화 로봇은 대부분 AGV(무인운반차) 기반의 단순 운반 기능에 초점을 맞추고 있어, 상품을 직접 집어 드는 피킹 기능이 포함되지 않았다. 이로 인해 자동화 수준이 한정적이며, 여전히 사람이 개입해야 하는 비효율적인 구조를 보였다.
이에 본 프로젝트에서는 기존 AGV 기반 로봇의 한계를 보완하고자, 피킹 기능을 포함한 자동 주문 피킹 로봇을 개발하였다. 사용자는 주문만 입력하면 로봇이 스스로 상품을 찾아 집어 옮기며, 인력 의존도를 낮추고 물류 효율을 극대화할 수 있다.

**1-3. 프로젝트 특장점**
- 기존 AGV 기반 단순 운반 로봇에서 발전한 로봇팔 결합형 자동 피킹 기능으로 완전한 물류 자동화 실현
- ArUco 마커 기반 정밀 위치 인식을 통해 ±1cm 오차 범위 내의 정확한 주행 및 선반 접근 실현
- OpenCV 기반 실시간 비전 인식을 통해 다양한 크기와 형태의 상품을 빠르고 정확하게 식별 및 파지
- 모듈형 구조 설계(차체, 리프트, 로봇팔, 비전 모듈)로 유지보수 용이성과 확장성 확보
- ROS 2 및 MQTT 통신 구조를 활용하여 주문–피킹–운반 과정을 실시간으로 연동한 통합 제어 시스템 구축
- 
**1-4. 주요 기능**
- 자동 피킹 기능 : 사용자의 주문 정보를 기반으로 로봇이 상품의 위치를 인식하고 자동으로 피킹 수행
- 정밀 위치 인식 : ArUco 마커를 활용하여 로봇의 현재 위치와 방향을 ±1cm 오차 이내로 추정
- 비전 기반 물체 인식 : OpenCV를 이용해 다양한 형태의 상품을 실시간으로 식별하고 중심 좌표 및 회전각 계산
- 역기구학 기반 로봇팔 제어 : 계산된 좌표를 바탕으로 각 관절의 최적 각도를 산출하여 부드럽고 안정적인 파지 동작 수행
- 실시간 통신 및 제어 연동 : MQTT 프로토콜을 통해 주문 앱–로봇 간 데이터를 실시간 교환하고, ROS 2 기반으로 각 모듈을 통합 제어

**1-5. 기대 효과 및 활용 분야**
- 기대 효과 : 물류센터의 자동화를 통해 인력 의존도 감소 및 작업 효율 향상, 피킹 오류 최소화로 정확한 주문 처리 실현, 24시간 무인 운영을 통한 운영비 절감 및 생산성 극대화
- 활용 분야 : 물류·유통 센터의 자동 피킹 시스템, 중소형 창고의 스마트 물류 구축, 무인 매장 재고 관리 로봇, 제조 라인의 부품 자동 공급 시스템, 스마트 팩토리 자동화 솔루션

---

## **💡2. 팀원 소개**
| <img width="80" height="100" src="https://github.com/user-attachments/assets/ab73bb1c-c1d4-464d-8ad3-635b45d5a8ae" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c7f66b7c-ab84-41fa-8fba-b49dba28b677" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c33252c7-3bf6-43cf-beaa-a9e2d9bd090b" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/0d5909f0-fc73-4ab9-be09-4d48e3e71083" > | <img width="80" height="100" alt="image" src="https://github.com/user-attachments/assets/c7f66b7c-ab84-41fa-8fba-b49dba28b677" > |
|:---:|:---:|:---:|:---:|:---:|
| **권태수** | **김상준** | **원태연** | **이재웅** | **박** |
| • ArUco 마커 기반 주행 알고리즘 개발 <br> • ROS2 메인 제어 시스템 통합 <br> • 전체 시스템 설계 및 통신 구조 구현| • OpenCV 기반 객체 인식 및 좌표 추출 <br> • 리프트·로봇팔 제어 알고리즘 개발 <br> • Android 앱(MQTT 통신) 제작| • 차체·리프트 기구 설계 및 제작 <br> • 모터 드라이버·센서 배선 구성 |• 전원 및 회로 배선 구성 <br> • 리프트·로봇팔 프레임 설계 및 제작 | • 프로젝트 멘토 <br> • 기술 자문 |




---
## **💡3. 시스템 구성도**

- 시스템 구성도도
<img width="695" height="572" alt="image" src="https://github.com/user-attachments/assets/b173342b-a108-4cbf-b5fb-85bbd89bf9b0" />


- 플로우 차트
<img width="695" height="572" alt="image" src="https://github.com/user-attachments/assets/604e2391-48dd-4c9d-9e32-73d98ed06720" />



---
## **💡4. 작품 소개영상**

[![한이음 드림업 프로젝트 소개](https://youtu.be/oBhbYQAAHrw?si=djJINzoDOdAHgMP9))


---
## **💡5. 핵심 소스코드**
- 소스코드 설명 : 리프트 동작및 노이즈 제거를 위한 EMA 필터 적용 코드입니다.

```
// 리프트 이동 함수
void moveToFloor(int targetFloor) {
  // 이미 같은 층이면 동작 불필요
  if (targetFloor == currentFloor) return;

  isMovingElevator = true; // 이동 중 상태로 설정

  // 이동 방향 결정 (상승: HIGH, 하강: LOW)
  digitalWrite(ELEV_DIR, (targetFloor > currentFloor) ? HIGH : LOW);

  // 목표층 도달 시까지 반복
  while (isMovingElevator) {
    generateElevPulse();    // 모터에 펄스 신호 전송
    updateCurrentFloor();   // 센서 상태로 현재층 갱신

    // 목표층에 도착하면 이동 정지
    if (currentFloor == targetFloor) {
      isMovingElevator = false;
    }
  }
}

// 펄스 생성 함수 (Duty 50%)
void generateElevPulse() {
  int period_us = 1000000 / elevFrequency; // 주기 계산 (1초 / 주파수)
  digitalWrite(ELEV_PUL, HIGH);            // HIGH 출력
  delayMicroseconds(period_us / 2);        // 절반 동안 유지
  digitalWrite(ELEV_PUL, LOW);             // LOW 출력
  delayMicroseconds(period_us / 2);        // 나머지 절반 동안 유지
}

// 현재 층 감지 함수
void updateCurrentFloor() {
  for (int i = 0; i < 4; i++) {
    // 센서가 LOW면 해당 층 감지 (트리거 방식)
    if (digitalRead(floorSensorPins[i]) == LOW) {
      currentFloor = i + 1; // 배열 인덱스(0~3)를 층 번호(1~4)로 변환
      break;                // 감지된 층이 있으면 루프 종료
    }
  }
}

// Y축 센서 EMA 필터 적용 함수
void updateAndPrintYSensors() {
  for (int i = 0; i < 4; i++) {
    int raw = digitalRead(ySensorPins[i]);          // 원본 센서 입력값 읽기
    float signal = (raw == LOW) ? 0.0 : 1.0;        // LOW=0, HIGH=1로 변환

    // EMA 필터 적용: 새 값보다 이전 값에 더 큰 가중치(α) 부여
    filteredY[i] = alpha * filteredY[i] + (1 - alpha) * signal;

    // 디버깅용 출력
    Serial.print("Y센서 "); Serial.print(i);
    Serial.print(": "); Serial.println(filteredY[i]);
  }
}

// 필터 결과를 HIGH/LOW로 반환
int getFilteredYState(int i) {
  // 필터 결과가 threshold보다 낮으면 LOW, 높으면 HIGH
  return (filteredY[i] < threshold) ? LOW : HIGH;
}

```
