import pandas as pd
import joblib
import json
from sklearn.linear_model import LinearRegression

# JSON 파일로부터 데이터 프레임 A를 읽어옵니다.
file_path_A = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/way_point_1.json' 
# df_A = pd.read_json(file_path_A)

# JSON 파일로부터 데이터 프레임 B를 읽어옵니다.
file_path_B = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/way_point_2.json' 
# df_B = pd.read_json(file_path_B)
# df_B.columns = ['output_1', 'output_2', 'output_3', 'output_4', 'output_5', 'output_6']

with open(file_path_A, 'r') as json_file:
    way_point_1 = json.load(json_file)

with open(file_path_B, 'r') as json_file:
    way_point_2 = json.load(json_file)


X = way_point_2.copy()
y = way_point_1.copy()
model = LinearRegression()
model.fit(X, y)

model_filename = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/linear_model_3.pkl'
joblib.dump(model, model_filename)




#################################################################################################
# model = LinearRegression()
# # 한줄씩 학습하는 코드
# predicted_rows = []
# for i in range(len(df_A)):
#     X = df_B.iloc[i:i+1, 1:4]  # 데이터 프레임 B의 i번째 행을 입력으로 선택
#     y = df_A.iloc[i:i+1, 1:4]  # 데이터 프레임 A의 i번째 행을 예측 대상으로 선택

#     model.fit(X, y) 
#     predicted_row = model.predict(X)  # 해당 행을 예측
#     predicted_rows.append(predicted_row)

# # 모델 저장
# model_filename = '/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/linear_model.pkl'
# joblib.dump(model, model_filename)

#################################################################################################
