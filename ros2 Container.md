### Ros2 컨테이너 사용법

**컨테이너?**

: 우분투 20.04버전에서 22.04 버전을 사용하기 위한 툴

**왜 사용하나?**

: 최신 릴리스 된 ros2 humble이 22.04 버전밖에 지원하지 않음 (2022/06/15 기준)



#### 방법

---

* lxd 설치 : 리눅스 커널의 컨테이너를 관리하는 모듈

```bash
$ sudo snap install lxd 
```

* 권한 설정

```bash
$ sudo usermod -a -G lxd $USER
```

* 컨테이너 생성 ( lxc launch <image_server>:<image_name <instance_name>)

```bash
$ lxc launch images:ubuntu/22.04 ubuntu-container
```

* 컨테이너 활성화

```bash
$ lxc exec ubuntu-container -- /bin/bash
```



