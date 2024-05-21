#!/bin/bash

# 将校园网添加到当前用户
# 每隔10min自动执行登陆脚本

# 这里填写你的自动登陆脚本的路径
autoLogin="/home/myx/develop/script/autoLogin.sh"

chmod +x "$autoLogin"

cron_entry="*/1 * * * * $autoLogin"

crontab -l > mycron

echo "$cron_entry" >> mycron

crontab mycron

rm mycron

echo "Add successfully: $(crontab -l)."
