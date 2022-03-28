'''
Author: wangxy
Date: 2022-03-26 15:26:23
LastEditTime: 2022-03-28 21:00:05
LastEditors: wangxy
Description: none
FilePath: /src/celex_kinect/script/event_time_statis.py
'''

from audioop import avg
import re
from time import time


# 获取总时间
def find_key_info(context, pattern):
    ret = []
    # 排除掉第一个，不知道为啥数值那么大
    isNotFirst = False
    for line in context:
        t = re.findall(pattern, line)
        if(len(t)):
            if not isNotFirst:
                isNotFirst = True
                continue
            if isNotFirst:
                ret.append(eval(t[0]))
    return ret


def cal_avg_time(time_list):
    if(len(time_list))==0:
        return 0
    total_time = 0
    for t in time_list:
        total_time += t
    avg_time = total_time/len(time_list)
    return avg_time


def main():
    file_path = eval(input('please drag the file here'))
    with open(file_path) as f:
        file_context = f.readlines()

    gen_time_list = find_key_info(file_context, r'generate cnt img: (.*)ms')
    detect_time_list = find_key_info(file_context, r'edge detect: (.*)ms')
    empty_time_list = find_key_info(file_context, r'empty detect: (.*)ms')
    timegen_list = find_key_info(file_context, r'timegen: (.*)ms')

    # print(gen_time_list)

    # for line in file_context:
    #     gen_cnt_img_time = re.findall(r'generate cnt img: (.*)ms', line)
    #     detect_time = re.findall(r'edge detect: (.*)ms', line)
    #     empty_detect_time = re.findall(r'empty detect: (.*)ms', line)
    #     if(len(gen_cnt_img_time)):
    #         gen_time_list.append(eval(gen_cnt_img_time[0]))
    #     if(len(detect_time)):
    #         detect_time_list.append(eval(detect_time[0]))
    #     if(len(empty_detect_time)):
    #         empty_time_list.append(eval(empty_detect_time[0]))

    # total_gen_time = 0
    # total_detect_time = 0
    # total_empty_time = 0
    # for t in gen_time_list:
    #     total_gen_time += t
    # for t in detect_time_list:
    #     total_detect_time += t
    # for t in empty_time_list:
    #     total_empty_time += t

    avg_gen_time = cal_avg_time(gen_time_list)
    avg_detect_time = cal_avg_time(detect_time_list)
    avg_empty_time = cal_avg_time(empty_time_list)
    timegen_time = cal_avg_time(timegen_list)

    print('avg time is:\ngen cnt img:{}\ndetect:{}\nempty:{}'.format(
        avg_gen_time, avg_detect_time-avg_gen_time, avg_empty_time))
    print("timegen time",timegen_time)


def testre():
    a = 'abcdefg'
    ret = re.findall(r'a(.*)a', a)
    print(ret)


if __name__ == '__main__':
    main()
    # testre()
