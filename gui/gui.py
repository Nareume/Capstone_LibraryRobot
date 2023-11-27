import tkinter as tk
from tkinter import PhotoImage
from PIL import Image, ImageTk

# 이미지의 크기를 변경하는 함수pip
def resize_image(image, new_width, new_height):
    image = image.resize((new_width, new_height), Image.ANTIALIAS)
    return ImageTk.PhotoImage(image)

# 반납 버튼 클릭 시 호출될 함수
def return_books():
    print("반납 모드를 시작합니다.")
    # 여기에 새 창을 여는 등의 로직을 구현할 수 있습니다.

# 안내 버튼 클릭 시 호출될 함수
def guide():
    print("안내 모드를 시작합니다.")
    # 여기에 새 창을 여는 등의 로직을 구현할 수 있습니다.

# 메인 윈도우 생성
root = tk.Tk()
root.title("도서관 책 정리 및 안내 시스템")

# 윈도우 크기 설정 (필요한 경우 조절)
root.geometry("1024x768")

# 배경 이미지 로드 (이미지 파일 경로에 맞게 수정)
background_image = Image.open("background_image1.png")  # 배경 이미지 파일 경로로 변경
background_photo = ImageTk.PhotoImage(background_image)
background_label = tk.Label(root, image=background_photo)
background_label.place(relwidth=1, relheight=1)


# 이미지 로드 및 크기 조정 (이미지 파일 경로에 맞게 수정)
return_image_original = Image.open("return_image.png")  # 반납 아이콘 이미지 파일 경로로 변경
guide_image_original = Image.open("guide_image.png")    # 안내 아이콘 이미지 파일 경로로 변경

# 이미지 크기 조정 (여기서는 예시로 100x100 크기를 사용합니다)
return_image_resized = resize_image(return_image_original, 250, 250)
guide_image_resized = resize_image(guide_image_original, 250, 250)

# 버튼 크기와 위치를 설정합니다.
button_width = 250
button_height = 250

# 반납 버튼 생성, 텍스트와 이미지 결합
return_button = tk.Button(root, image=return_image_resized, compound="top", bg='white', width=button_width, height=button_height, command=return_books)
return_button.place(x=150, y=350)  # 위치는 윈도우 크기와 버튼 크기에 맞춰 조정

# 안내 버튼 생성, 텍스트와 이미지 결합
guide_button = tk.Button(root, image=guide_image_resized, compound="top", bg='white', width=button_width, height=button_height, command=guide)
guide_button.place(x=650, y=350)  # 위치는 윈도우 크기와 버튼 크기에 맞춰 조정

# 메인 루프 시작
root.mainloop()
