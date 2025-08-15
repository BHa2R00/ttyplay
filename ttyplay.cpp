#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Widget.H>
#include <FL/fl_draw.H>
#include <FL/fl_ask.H>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
#include <cmath>
#include <alsa/asoundlib.h>
#include <errno.h>
#include <cctype>

#define PORT "/dev/ttyUSB0"
#define BAUDRATE B1152000
#define BUFFER_SIZE 1000
#define WINDOW_WIDTH BUFFER_SIZE
#define WINDOW_HEIGHT 400
#define SAMPLE_RATE 24000
#define PERIOD_SIZE BUFFER_SIZE 
#define AUDIO_THREAD_PRIORITY 50 

// 全局数据（由互斥锁保护）
short waveform[BUFFER_SIZE] = {0};
int buffer_idx = 0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
bool running = true;
snd_pcm_t *pcm_handle = nullptr;  // ALSA音频设备句柄

// 串口初始化（保持不变）
int setup_serial() {
    int fd = open(PORT, O_RDONLY | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) return -1;

    // 设置波特率
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    // 8N1配置
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // 原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return fd;
}

// ALSA音频设备初始化（保持不变）
int setup_audio() {
    int rc;
    snd_pcm_hw_params_t *params;
    unsigned int val;
    int dir;
    snd_pcm_uframes_t frames;

    // 打开PCM设备（默认音频输出）
    rc = snd_pcm_open(&pcm_handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    /*if (rc < 0) {
        fl_alert("无法打开PCM设备: %s", snd_strerror(rc));
        return -1;
    }*/

    // 分配硬件参数对象
    snd_pcm_hw_params_alloca(&params);

    // 填充默认参数
    snd_pcm_hw_params_any(pcm_handle, params);

    // 设置参数
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);  // 16位有符号整数
    snd_pcm_hw_params_set_channels(pcm_handle, params, 1);  // 单声道

    val = SAMPLE_RATE;
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &val, &dir);

    frames = PERIOD_SIZE;
    snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &frames, &dir);

    // 应用参数
    rc = snd_pcm_hw_params(pcm_handle, params);
    /*if (rc < 0) {
        fl_alert("无法设置硬件参数: %s", snd_strerror(rc));
        return -1;
    }*/

    return 0;
}

// 将4字符ASCII十六进制（如"7FFF"）转换为16位有符号整数（保持不变）
short ascii_to_sample(const char* ascii) {
    char* endptr;
    long val = strtol(ascii, &endptr, 16);
    if (*endptr != '\0' || val < 0 || val > 0xFFFF) return 0;
    // 转换为有符号16位整数
    return (val > 32767) ? val - 65536 : val;
}

// UART读取线程（使用scanf修改版）
void* read_thread(void* arg) {
    int fd = *(int*)arg;
    
    // Redirect serial port to stdin for scanf
    dup2(fd, STDIN_FILENO);
    
    unsigned int raw_sample;  // Use unsigned to handle 0x0000-0xFFFF
    
    while (running) {
        // Read 4-digit hex value with optional leading whitespace
        int ret = scanf(" %04x", &raw_sample);
        
        if (ret == 1) {  // Successful read
            // Convert to signed 16-bit correctly
            short sample = (raw_sample > 32767) ? raw_sample - 65536 : raw_sample;
            sample = sample << 6;  // Amplify signal (keep as needed)
            
            // Update waveform buffer
            pthread_mutex_lock(&mutex);
            waveform[buffer_idx] = sample;
            buffer_idx = (buffer_idx + 1) % BUFFER_SIZE;
            pthread_mutex_unlock(&mutex);
        }
        else if (ret == EOF) {  // End of stream or error
            perror("Serial read error (EOF)");
            running = false;
            break;
        }
        else {  // Invalid input (ret == 0)
            // Clear invalid characters from buffer
            fprintf(stderr, "Invalid input, clearing buffer\n");
            
            // Read and discard up to 100 characters or until newline/whitespace
            int c;
            int discard_count = 0;
            while ((c = getchar()) != EOF && discard_count < 100) {
                discard_count++;
                if (isspace(c)) break;  // Stop at next whitespace
            }
            
            // If we hit EOF during clearing
            if (c == EOF) {
                perror("Serial read error during buffer clear");
                running = false;
                break;
            }
        }
    }
    return NULL;
}

// 音频播放线程（优化数据填充逻辑）
void* audio_play_thread(void* arg) {
    short audio_buf[PERIOD_SIZE];
    int audio_idx = 0;

    while (running) {
        pthread_mutex_lock(&mutex);
        
        // 1. 保存当前buffer索引快照，避免复制过程中被修改
        int current_buf_idx = buffer_idx;
        // 2. 计算起始位置
        int start_idx = (current_buf_idx + audio_idx) % BUFFER_SIZE;
        // 3. 高效复制数据（处理环形缓冲区边界情况）
        if (start_idx + PERIOD_SIZE <= BUFFER_SIZE) {
            // 无需跨边界，直接复制
            memcpy(audio_buf, &waveform[start_idx], PERIOD_SIZE * sizeof(short));
        } else {
            // 跨边界情况，分两段复制
            int first_part = BUFFER_SIZE - start_idx;
            memcpy(audio_buf, &waveform[start_idx], first_part * sizeof(short));
            memcpy(audio_buf + first_part, &waveform[0], 
                   (PERIOD_SIZE - first_part) * sizeof(short));
        }
        // 4. 更新音频索引
        audio_idx = (audio_idx + PERIOD_SIZE) % BUFFER_SIZE;
        
        pthread_mutex_unlock(&mutex);  // 尽早释放锁，减少阻塞

        // 播放音频
        int rc = snd_pcm_writei(pcm_handle, audio_buf, PERIOD_SIZE);
        if (rc == -EPIPE) {
            // 发生溢出，恢复
            snd_pcm_prepare(pcm_handle);
        } else if (rc < 0) {
            fl_alert("写入PCM设备失败: %s", snd_strerror(rc));
        } else if (rc != PERIOD_SIZE) {
            fl_alert("写入的样本数不正确: 预期 %d, 实际 %d", PERIOD_SIZE, rc);
        }
    }
    return nullptr;
}

// 自定义FLTK组件：绘制波形（保持不变）
class WaveformWidget : public Fl_Widget {
public:
    WaveformWidget(int x, int y, int w, int h) : Fl_Widget(x, y, w, h) {}

    void draw() override {
        // 清除背景（白色）
        fl_color(FL_WHITE);
        fl_rectf(x(), y(), w(), h());

        // 绘制网格
        fl_color(FL_LIGHT3);
        int grid_spacing = h() / 5;
        for (int i = 1; i < 5; ++i) {
            int grid_y = y() + i * grid_spacing;
            fl_line(x(), grid_y, x() + w(), grid_y);
        }

        // 绘制波形（蓝色）
        fl_color(FL_BLUE);
        fl_line_style(FL_SOLID, 2);

        pthread_mutex_lock(&mutex);
        int x_step = w() / BUFFER_SIZE;
        for (int i = 0; i < BUFFER_SIZE - 1; ++i) {
            int idx1 = (buffer_idx + i) % BUFFER_SIZE;
            int idx2 = (buffer_idx + i + 1) % BUFFER_SIZE;

            // 将样本值转换为Y坐标（适配窗口高度）
            int y1 = y() + h() - ((waveform[idx1] + 32768) * h() / 65536);
            int y2 = y() + h() - ((waveform[idx2] + 32768) * h() / 65536);

            // 绘制线段
            fl_line(x() + i * x_step, y1, x() + (i + 1) * x_step, y2);
        }
        pthread_mutex_unlock(&mutex);

        fl_line_style(FL_SOLID, 1);  // 重置线样式
    }
};

// 主窗口（添加音频线程优先级设置）
class MainWindow : public Fl_Window {
public:
    MainWindow(int w, int h, const char* title) : Fl_Window(w, h, title) {
        waveform_widget = new WaveformWidget(0, 0, w, h);
        end();  // 结束窗口定义

        // 设置定时器刷新波形（根据周期大小调整刷新频率）
        Fl::add_timeout((double)PERIOD_SIZE / SAMPLE_RATE, timer_callback, this);
    }

    ~MainWindow() {
        running = false;
        pthread_join(serial_thread, nullptr);
        pthread_join(audio_thread, nullptr);  // 等待音频线程结束
        close(serial_fd);
        if (pcm_handle) {
            snd_pcm_drain(pcm_handle);  // 排空并关闭音频设备
            snd_pcm_close(pcm_handle);
        }
        pthread_mutex_destroy(&mutex);
    }

    int setup() {
        // 打开串口
        serial_fd = setup_serial();
        if (serial_fd < 0) {
            fl_alert("无法打开串口 %s", PORT);
            return 1;
        }

        // 初始化音频设备
        if (setup_audio() < 0) {
            //close(serial_fd);
            return 1;
        }

        // 启动读取线程
        if (pthread_create(&serial_thread, nullptr, read_thread, &serial_fd) != 0) {
            fl_alert("无法创建串口读取线程");
            //close(serial_fd);
            snd_pcm_close(pcm_handle);
            return 1;
        }

        // 启动音频播放线程
        if (pthread_create(&audio_thread, nullptr, audio_play_thread, nullptr) != 0) {
            fl_alert("无法创建音频播放线程");
            running = false;
            pthread_join(serial_thread, nullptr);
            //close(serial_fd);
            snd_pcm_close(pcm_handle);
            return 1;
        }

        // 提升音频线程优先级
        if (set_thread_priority(audio_thread, AUDIO_THREAD_PRIORITY) != 0) {
            fl_alert("警告: 无法设置音频线程优先级，可能导致播放卡顿\n请确保程序有足够权限(CAP_SYS_NICE)");
        }

        return 0;
    }

private:
    WaveformWidget* waveform_widget;
    int serial_fd;
    pthread_t serial_thread;
    pthread_t audio_thread;  // 音频播放线程

    // 设置线程优先级的工具函数
    int set_thread_priority(pthread_t thread, int priority) {
        struct sched_param param;
        int policy;
        
        // 获取当前调度策略
        if (pthread_getschedparam(thread, &policy, &param) != 0) {
            //fl_alert("获取线程调度参数失败: %s", strerror(errno));
            return -1;
        }
        
        // 设置实时调度策略和优先级
        policy = SCHED_FIFO;  // 实时先进先出调度
        param.sched_priority = priority;
        
        // 检查优先级范围
        int min_prio = sched_get_priority_min(policy);
        int max_prio = sched_get_priority_max(policy);
        if (priority < min_prio || priority > max_prio) {
            //fl_alert("优先级超出范围 [%d, %d]", min_prio, max_prio);
            return -1;
        }
        
        // 应用调度参数
        if (pthread_setschedparam(thread, policy, &param) != 0) {
            //fl_alert("设置线程优先级失败: %s", strerror(errno));
            return -1;
        }
        
        return 0;
    }

    static void timer_callback(void* data) {
        MainWindow* win = (MainWindow*)data;
        win->waveform_widget->redraw();  // 触发重绘
        Fl::repeat_timeout((double)PERIOD_SIZE / SAMPLE_RATE, timer_callback, data);
    }
};

int main() {
    MainWindow win(WINDOW_WIDTH, WINDOW_HEIGHT, "UART波形监控与播放");
    if (win.setup() != 0) return 1;
    win.show();
    return Fl::run();
}


// 编译命令需要添加ALSA库链接
// g++ ttyplay.cpp -I/usr/include/fltk -L/usr/lib64/fltk -lfltk -lpthread -lasound -o ttyplay
