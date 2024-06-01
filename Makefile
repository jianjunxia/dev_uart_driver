.PHONY:all

#获取当前目录下的c文件集
CUR_CFILES=${wildcard *.c}

#将对应的c文件名转为o文件
CUR_OFILES=${patsubst %.c, %.o, $(CUR_CFILES)}

# 加入 -Wall -Wextra -Werror -fstack-protector 标志到 CFLAG 中, 警告当作错误处理,增强代码质量
CFLAG += -Wall -Wextra -Werror -fstack-protector -Wall -Wextra -Werror -fstack-protector

all:$(CUR_OFILES)

$(CUR_OFILES):%.o:%.c
	@$(CC) -c $^ -o $(LIN_DIR)/$@ $(CFLAG)
	@echo -e "\033[36m    >> CC[ $(CC) ] compile $(notdir $<) ......\033[0m"
	