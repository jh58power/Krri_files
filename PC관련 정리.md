## 실습생 5 PC 관련 정리

* 비밀번호 : krri1234!
* 원내망 IP : 10.10.33.82
* NAS IP : 192.168.2.105
* NAS 주소 : 192.168.2.84 (username = 'student', pw = 'rr1234!')
* 팀뷰어 PW : 12345678

#### HDD 마운트

```bash
sudo mount /dev/sdc1 /home/krri/HDD
sudo mount /dev/sda /home/krri/SSD
```

#### NAS 폴더 마운트

```bash
sudo mount -t cifs //192.168.2.94/student/ /home/krri/nas_student -o user='student',password='rr1234!',rw,vers=1.0
sudo mount -t cifs //192.168.2.94/brtdata/ /home/krri/brtdata -o user='student',password='rr1234!',rw,vers=1.0
```

#### NAS 자동 마운트 설정

/etc/fstab 에 마운트 정보 등록

//192.168.2.94/student /home/krri/nas_student cifs user='student,pass='rr1234!',rw,vers=1.0 0 0

//192.168.2.94/team/ /home/krri/nas_team cifs user='student,pass='rr1234!',rw,vers=1.0 0 0









