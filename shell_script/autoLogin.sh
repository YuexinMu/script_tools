#!/bin/bash

# 登陆校园网脚本
# 这里填写你的校园网用户名和密码
user_id=202324021066T
password=myx05159816

ping -c 1 -W 1 baidu.com > /dev/null 2>&1

if [ $? -ne 0 ]; then
curl '10.254.7.4' -d "DDDDD=${user_id}&upass=${password}&0MKKey=" > /dev/null 2>&1
fi

#echo "DDDDD=${user_id}&upass=${password}&0MKKey="
#echo "hello autoLogin"

