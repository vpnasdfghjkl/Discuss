
import json

# # print("yellow block","cx:",boxes[0,1]-0.3184,"cy:",boxes[0,0]-0.1206)
# # print("bowl","cx:",boxes[0,1]-0.2684, "cy:",boxes[0,0]+0.05)
# print(end-begin)
# # cv2.imwrite("false/false_5mix_color_mug.jpg", annotated_frame)
#
# cv2.imwrite("annotated7.jpg", annotated_frame)
# img = cv2.imread("annotated7.jpg")
# cv2.imshow("image",img)
# cv2.waitKey(0)



import requests
import os
def upload_image(image_path,content):
    """上传图片接口"""

    url = "http://192.168.1.205:5000/GRD"
    file_name = os.path.basename(image_path)
    data = {"question": "*"+content}  ##传入的文本指令

    with open(image_path,'rb')as f:
        files = {'file': (file_name, f.read(), 'image/jpg')}
    response = requests.post(url, data=data,files=files)
    print(response.text)

if __name__=='__main__':
    # 上传图片
    # sendImg(img_path="dataset/block/3_cube_size.png",content="粉红色的方块")

    upload_image(image_path="/home/woosh/catkin_ws/src/demo/scripts/cube.png",content="pink cube")# 调用sendImg方法


