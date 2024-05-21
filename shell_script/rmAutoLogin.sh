#!/bin/bash

# 删除自动执行登陆的脚本

# 这里填写你的自动登陆脚本的路径
autoLogin=/home/myx/develop/script/autoLogin.sh


rmcron="*/1 * * * * $autoLogin"

#echo "rmcron:${rmcron}"
current_crontab=$(crontab -l)

#echo "current_crontab: ${current_crontab}"
echo "$current_crontab" | grep -v "$rmcron" >> mycron

# echo "new_crontab: ${new_crontab}"

echo "$mycron" | crontab -

$(crontab -l)
echo "Remove successfully."

rm mycron
