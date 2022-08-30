import numpy as np
import pandas as pd
import sqlite3
import os
import glob
'''
사용방법

1. Create EmptyBag : Ros2 bag 형식의 DB3 파일을 만든다.

2. bagSplit(원본 Bag 경로(폴더명이여야함), OUTPUT 파일 경로 + 파일 명, 시작 프레임, 끝 프레임, 오프셋)
  - 프레임은 10hz 기준으로 0.1 초마다 취득한 데이터이다.
  - offset 은 프레임 누락, 반올림 등의 오차로 인한 데이터 손실을 방지하여 앞 뒤로 3초씩 추가하는 설정

코드 동작 방식은

    - 원본 Bag 폴더에 있는 .db3 파일들 모두 불러와 timestamp 배열 생성
    - 시작프레임 * 10000000 하여 시작 timestamp 위치 계산 (끝 프레임도 동일)
    - "SELECT * FROM messages WHERE timestamp >= {startTime} AND timestamp < {endTime}" 
      - 원본 Bag 에서 추출한 timstamp 범위의 값 불러옴
    - INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?,?,?,?);
      - 불러온 값들을 위 Create Empty Bag 을 통해 만든 빈 Bag db3 에 Insert
      - 단, topic_id 값이 모든 bag 마다 다르기 때문에 기준이 되는 bag 의 id,name 을 딕셔너리로 설정후 매칭
    - Commit
    
'''
# 빈 테이블 만들기
def create_EmptyBag(bag_name):
    #Table 및 Column 명 타입 지정
    try:
        print("making empty bag")
        
        assert not os.path.exists(bag_name)
        
        con_dst = sqlite3.connect(bag_name)
        cur_dst = con_dst.cursor()
        cur_dst.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
        cur_dst.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL)')

        #topic 명들 지정
        con_dst.commit()
    except:
        print("EmptyBag Exist")
        pass
        
# 빈 테이블에 데이터 추가        
def bagSplit(bag_path, dst_bag,startFrame,endFrame,offset):   
    bag_path_len = len(glob.glob(bag_path + "/" + bag_path.split("/")[-1]+"_*.db3"))
    
    con_dst = sqlite3.connect(dst_bag)
    cur_dst = con_dst.cursor()
        
    #frame 단위가 정확하지 않기 때문에 앞 뒤로 offset 추가
    startFrame = startFrame - offset
    endFrame = endFrame + offset
    
    print(bag_path + "/" + bag_path.split("/")[-1]+"_0.db3")
    con = sqlite3.connect(bag_path + "/" + bag_path.split("/")[-1]+"_0.db3")
    cur = con.cursor()
    
    criteria_topic = {}
    
    # 기준이 되는 topic 테이블 및 딕셔너리 생성
    sql_t = "INSERT INTO topics(id, name,type, serialization_format, offered_qos_profiles) VALUES (?,?,?,?,?);"
    for row in cur.execute("SELECT * FROM topics"):
        criteria_topic[row[1]] = row[0]
        cur_dst.execute(sql_t,(row[0],row[1],row[2],row[3],row[4]))
        
    for time in cur.execute('SELECT timestamp FROM messages'):
        startTime = time[0] + startFrame*100000000
        endTime = time[0] + endFrame*100000000   
        break

    print("Reading Bag")
    
    # Bag 이 여러개 나누어져 있는 경우
    sql_m = "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?,?,?,?);"
    
    refer_topic = {}
    for i in range(bag_path_len):
        con = sqlite3.connect(bag_path + "/" + bag_path.split("/")[-1]+f"_{i}.db3")
        cur = con.cursor()
        #message 테이블 값
        print(bag_path.split("/")[-1] + f"_{i}.db3",end="\r")
        
        for row in cur.execute("SELECT * FROM topics"):
            refer_topic[row[0]] = criteria_topic[row[1]]
            
        
        # id, topic_id, timestamp, data
        for row in cur.execute(f"SELECT * FROM messages WHERE timestamp >= {startTime} AND timestamp < {endTime}"):
            cur_dst.execute(sql_m,(row[0],refer_topic[row[1]],row[2],row[3]))


    cur_dst.execute('CREATE INDEX timestamp_idx ON messages (timestamp ASC)')
    print("commit")
    con_dst.commit()

    con.close()
    con_dst.close()
    

def bagMerge(bag1, bag2, dst_bag):

    con1 = sqlite3.connect(bag1)
    cur1 = con1.cursor()

    con2 = sqlite3.connect(bag2)
    cur2 = con2.cursor()

    con_dst = sqlite3.connect(dst_bag)
    cur_dst = con_dst.cursor()


    criteria_topic = {}
    refer_topic = {}


    #기준이 되는 토픽 테이블 딕셔너리 생성
    for id,name in cur1.execute("SELECT id, name FROM topics"):
        criteria_topic[name] = id
        
    #대응하는 id 딕셔너리 생성
    for id,name in cur2.execute("SELECT id, name FROM topics"):
        refer_topic[id] = criteria_topic[name]  # 기준 토픽 name 에 해당하는 id 로 변환하기 위한 딕셔너리
    #timestamp
    prev_time = 0
    time_diff = []
    for time in cur2.execute("SELECT timestamp FROM messages"):
        time_diff.append(time[0] - prev_time)    
        prev_time = time[0]
    time_diff[0] = time_diff[1]

    #첫번째 Bag 기준으로 dst bag 에 topic 테이블 값 추가
    sql_t = "INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) VALUES (?,?,?,?,?);"
    for row in cur1.execute("SELECT * FROM topics"):
        cur_dst.execute(sql_t,(row[0],row[1],row[2],row[3],row[4]))
        
    #1번 Bag Insert
    print(bag1)
    sql_m = "INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?,?,?,?);"
    for row in cur1.execute("SELECT * FROM messages"):
        cur_dst.execute(sql_m,(row[0],row[1],row[2],row[3]))
        print(row[2],end='\r')
        last_time = row[2]
        last_id = row[0]
        
    print(bag2)
    #2번 Bag Insert
    i = 0
    last_id = last_id + 1
    for id,topic_id,timestamp,data in cur2.execute("SELECT * FROM messages"):
        cur_dst.execute(sql_m,(last_id,refer_topic[topic_id],last_time + time_diff[i],data))
        last_time = last_time + time_diff[i]
        last_id += 1
        i += 1
        print(row[2],end='\r')

    cur_dst.execute('CREATE INDEX timestamp_idx ON messages (timestamp ASC)')
    print("commit")
    con_dst.commit()

    con1.close()
    con2.close()
    con_dst.close()