### Uduntu 정리

### USB 포트 별칭 설정 방법

1. 장치 고유의 특징 찾기

   ```
   $ udevadm info -a -n /dev/ttyUSB* | grep '{serial}' 
   ```

    -> ttyUSB로 시작하는 장치들의 특징 중 serial에 해당하는 값만 grep

   -> VenderID와 ProductID 도 함께 검색 `(ID 0403:6001)`

   

2. UDEV 설정

   ```
   $ cd /etc/udev/rules.d
   ```

   안에 99-usb-serial.rules 라는 파일 생성

   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A6008isP", SYMLINK+="원하는이름"
   ```

   ->  위와 같이 필요한 정보들 입력

   

3. udev 재시작

   ```
   sudo service udev restart
   ```

4. 확인

   ```
   ls -al /dev/ttyUSB*
   ```

   

### 











