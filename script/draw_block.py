'''
Author: wangxy
Date: 2022-03-26 10:41:39
LastEditTime: 2022-03-27 10:41:59
LastEditors: wangxy
Description: none
FilePath: /src/celex_kinect/script/draw_block.py
'''
import cv2
import re
import os

def main():
    img_path=eval(input('drag img to here'))
    src_img=cv2.imread(img_path)

    
    start_row_point=[i*80 for i in range(1,10)]
    start_col_point=[i*80 for i in range(1,16)]
    
    for start_point in start_col_point:
        
        cv2.line(src_img,(start_point,0),(start_point,800),(0,255,0),2)

    for start_point in start_row_point:
        cv2.line(src_img,(0,start_point),(1280,start_point),(0,255,0),2)
    
    # cv2.imshow("src_img",src_img)
    # cv2.waitKey(5000)

    file_dir,file_name=os.path.split(img_path)
    save_path=os.path.join(file_dir,'draw_block_result.png')
    cv2.imwrite(save_path,src_img)



if __name__=='__main__':
    main()