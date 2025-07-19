# ───────────── 使用者可調整區 ─────────────

TARGET    := bh1750_daemon              # 最終執行檔名稱
SRCS      := main.c bh1750.c            # 所有 *.c 源碼
INCDIRS   := -Iinclude                  # header 搜尋路徑
CFLAGS    := -O2 -Wall $(INCDIRS)
LDFLAGS   := -lrt -pthread
INSTALLDIR?= /usr/local/bin             # Rock Pi 目標路徑
HOST      ?= rockpi@192.168.1.42        # scp 帳號@IP；依實際環境調整
# ────────────────────────────────────────

# ==== 通用設定，通常不用改 ====
CC   := $(CROSS_COMPILE)gcc
OBJS := $(SRCS:.c=.o)

.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# 產生 .o（pattern rule）
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install: $(TARGET)
	scp $< $(HOST):$(INSTALLDIR)/

clean:
	$(RM) $(OBJS) $(TARGET)