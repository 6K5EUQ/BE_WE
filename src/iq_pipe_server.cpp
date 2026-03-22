#include "iq_pipe_server.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>

int IqPipeServer::make_listen_sock(int port){
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0) return -1;
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);
    if(bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0){ close(fd); return -1; }
    if(listen(fd, 16) < 0){ close(fd); return -1; }
    return fd;
}

bool IqPipeServer::start(int port){
    listen_fd_ = make_listen_sock(port);
    if(listen_fd_ < 0){
        fprintf(stderr, "[IqPipeServer] listen on %d failed: %s\n", port, strerror(errno));
        return false;
    }
    running_.store(true);
    accept_thr_ = std::thread(&IqPipeServer::accept_loop, this);
    printf("[IqPipeServer] listening on port %d\n", port);
    return true;
}

void IqPipeServer::stop(){
    running_.store(false);
    if(listen_fd_ >= 0){ shutdown(listen_fd_, SHUT_RDWR); close(listen_fd_); listen_fd_=-1; }
    if(accept_thr_.joinable()) accept_thr_.join();
}

void IqPipeServer::register_send(uint32_t req_id, const std::string& file_path, ProgressCb cb){
    std::lock_guard<std::mutex> lk(entries_mtx_);
    // 이미 같은 req_id가 있으면 갱신
    for(auto& e : entries_){
        if(e.req_id == req_id){ e.file_path = file_path; e.cb = cb; return; }
    }
    SendEntry e;
    e.req_id    = req_id;
    e.file_path = file_path;
    e.cb        = cb;
    e.created   = std::chrono::steady_clock::now();
    entries_.push_back(std::move(e));
    printf("[IqPipeServer] registered req_id=%u file='%s'\n", req_id, file_path.c_str());
}

void IqPipeServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){
            if(running_.load()) perror("[IqPipeServer] accept");
            break;
        }
        int nd = 1; setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        int rbuf = 4*1024*1024, sbuf = 4*1024*1024;
        setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &rbuf, sizeof(rbuf));
        setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &sbuf, sizeof(sbuf));
        // 타임아웃 없음: 파일 전송 중 블로킹 허용
        timeval tv0{0,0};
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
        std::thread([this, cfd](){ handle_join(cfd); }).detach();
    }
}

void IqPipeServer::handle_join(int fd){
    // JOIN이 req_id(4바이트 LE)를 먼저 전송
    uint32_t req_id = 0;
    {
        timeval tv{10, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        ssize_t r = recv(fd, &req_id, 4, MSG_WAITALL);
        timeval tv0{0,0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
        if(r != 4){
            printf("[IqPipeServer] handle_join: req_id recv failed r=%zd\n", r);
            close(fd); return;
        }
    }
    printf("[IqPipeServer] JOIN connected req_id=%u\n", req_id);

    // entries_에서 매칭되는 항목 찾기 (최대 60초 대기)
    std::string file_path;
    ProgressCb  cb;
    for(int wait = 0; wait < 600; wait++){
        {
            std::lock_guard<std::mutex> lk(entries_mtx_);
            for(auto it = entries_.begin(); it != entries_.end(); ++it){
                if(it->req_id == req_id){
                    file_path = it->file_path;
                    cb        = it->cb;
                    entries_.erase(it);
                    break;
                }
            }
        }
        if(!file_path.empty()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(file_path.empty()){
        printf("[IqPipeServer] req_id=%u: no matching send entry (timeout)\n", req_id);
        close(fd); return;
    }

    // 파일 열기
    FILE* fp = fopen(file_path.c_str(), "rb");
    if(!fp){
        printf("[IqPipeServer] req_id=%u: cannot open '%s'\n", req_id, file_path.c_str());
        if(cb) cb(0, 0, -1);
        close(fd); return;
    }
    fseek(fp, 0, SEEK_END);
    uint64_t total = (uint64_t)ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if(cb) cb(0, total, 1);  // phase=1: 전송 시작

    const size_t CHUNK = 256 * 1024;
    std::vector<uint8_t> buf(CHUNK);
    uint64_t sent = 0;

    while(true){
        size_t n = fread(buf.data(), 1, CHUNK, fp);
        if(n == 0) break;

        // 전체 청크를 보장하며 전송
        const uint8_t* p = buf.data();
        size_t rem = n;
        while(rem > 0){
            ssize_t r = send(fd, p, rem, MSG_NOSIGNAL);
            if(r <= 0){
                printf("[IqPipeServer] req_id=%u: send failed r=%zd errno=%d(%s)\n",
                       req_id, r, errno, strerror(errno));
                fclose(fp); close(fd);
                if(cb) cb(sent, total, -1);
                return;
            }
            p += r; rem -= r; sent += (uint64_t)r;
        }

        if(cb) cb(sent, total, 1);
    }

    fclose(fp);
    close(fd);
    printf("[IqPipeServer] req_id=%u: done %.2f MB\n", req_id, sent/1048576.0);
    if(cb) cb(sent, total, 2);  // phase=2: 완료
}
