import joblib

# 모델을 불러오기
loaded_model = joblib.load("/home/seokwon/deeplearn/src/ai_manipulation/ai_manipulation/datas/linear_model_3.pkl")

# 새로운 입력 데이터
#new_X = [[-1.0906603336,0.5276893973,-0.7025632262,1.7763497829,-1.6,-0.7455146909]]
new_X = [[0.4678641558,0.7531846166,-1.0875923634,1.9466216564,1.6,-0.8160778284]]
# 저장된 모델을 사용하여 예측
predicted_y = loaded_model.predict(new_X)

# 예측값 출력
print("Predicted Value:", predicted_y)
