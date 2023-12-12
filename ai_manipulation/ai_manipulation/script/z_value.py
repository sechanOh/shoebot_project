import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split

# 데이터프레임 A와 B 로드
json_A_path = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/way_point_1.json'  # A 데이터 파일 경로
json_B_path = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/way_point_2.json'  # B 데이터 파일 경로

# JSON 파일을 데이터프레임으로 읽어오기
data_A = pd.read_json(json_A_path)
data_B = pd.read_json(json_B_path)

# 데이터 전처리
X = data_A.values  # A의 데이터
y = data_B.values  # B의 데이터

# 데이터 정규화 또는 스케일링
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
X = scaler.fit_transform(X)
y = scaler.fit_transform(y)

# 데이터 분할
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 모델 생성
input_dim = X.shape[1]
output_dim = y.shape[1]
model = keras.Sequential([
    keras.layers.Dense(64, activation='relu', input_shape=(input_dim,)),
    keras.layers.Dense(32, activation='relu'),
    keras.layers.Dense(output_dim)
])

# 모델 컴파일
model.compile(loss='mean_squared_error', optimizer='adam')

# 모델 학습
model.fit(X_train, y_train, epochs=1000, batch_size=32, validation_split=0.2)

# 모델 평가
test_loss = model.evaluate(X_test, y_test)
print(f'Test Loss: {test_loss}')

# 예측
y_pred = model.predict(X_test)



# 모델을 저장할 경로와 파일 이름 설정
model_path = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas'

# 모델을 파일로 저장
model.save(model_path)

