// DEPRECATED: native C web server (mongoose) is no longer used.
// The web UI is served by `webui/server.js` running on a lightweight NodeJS
// binary (see webui/ and `calib-web-node.bat`). Keep this file only as
// reference; do not extend or rebuild it.
#include "mongoose.h"
#include "cJSON.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <filesystem>
namespace fs = std::filesystem;

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd
#define strcasecmp _stricmp
#define popen _popen
#define pclose _pclose
#else
#include <unistd.h>
#endif

static const char *s_root_dir = "webui";
static const char *s_ssi_pattern = "#.html";

bool match_uri(struct mg_str uri, const char *path) {
    return uri.len == strlen(path) && memcmp(uri.buf, path, uri.len) == 0;
}

void handle_list(struct mg_connection *c, struct mg_http_message *hm) {
    struct mg_str dir_query = mg_http_var(hm->query, mg_str("dir"));
    std::string dir_param = ".output";
    
    if (dir_query.len > 0) {
        char decoded[512];
        // FIX 1: mg_url_decode does NOT null-terminate. We must do it manually.
        int len = mg_url_decode(dir_query.buf, dir_query.len, decoded, sizeof(decoded) - 1, 0);
        if (len >= 0) {
            decoded[len] = '\0';
            dir_param = decoded;
        }
    }

    cJSON *root = cJSON_CreateArray();

    try {
        if (fs::exists(dir_param) && fs::is_directory(dir_param)) {
            for (const auto& entry : fs::directory_iterator(dir_param)) {
                // Get filename as UTF-8 string
                std::string filename = entry.path().filename().string();
                
                cJSON *item = cJSON_CreateObject();
                cJSON_AddStringToObject(item, "name", filename.c_str());
                cJSON_AddBoolToObject(item, "isDirectory", entry.is_directory());
                
                // Get extension
                std::string ext = entry.path().extension().string();
                cJSON_AddStringToObject(item, "ext", ext.c_str());
                
                cJSON_AddItemToArray(root, item);
            }
        }
    } catch (const std::exception& e) {
        printf("FS Error: %s\n", e.what());
    }

    char *json_str = cJSON_Print(root);
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_str);
    
    free(json_str);
    cJSON_Delete(root);
}

void handle_run(struct mg_connection *c, struct mg_http_message *hm) {
    std::string body(hm->body.buf, hm->body.len);
    
    cJSON *root = cJSON_Parse(body.c_str());
    if (!root) {
        mg_http_reply(c, 400, NULL, "Invalid JSON");
        return;
    }

    std::string inDir = cJSON_GetObjectItem(root, "inDir") ? cJSON_GetObjectItem(root, "inDir")->valuestring : ".input";
    std::string outDir = cJSON_GetObjectItem(root, "outDir") ? cJSON_GetObjectItem(root, "outDir")->valuestring : ".output";
    bool doRadio = cJSON_GetObjectItem(root, "doRadio") ? cJSON_IsTrue(cJSON_GetObjectItem(root, "doRadio")) : false;
    int autoRadioThickness = cJSON_GetObjectItem(root, "autoRadioThickness") ? cJSON_GetObjectItem(root, "autoRadioThickness")->valueint : -1;
    std::string radioRefFile = cJSON_GetObjectItem(root, "radioRefFile") ? cJSON_GetObjectItem(root, "radioRefFile")->valuestring : "";
    std::string radioTemplatePath = cJSON_GetObjectItem(root, "radioTemplatePath") ? cJSON_GetObjectItem(root, "radioTemplatePath")->valuestring : "";
    bool optimize = cJSON_GetObjectItem(root, "optimize") ? cJSON_IsTrue(cJSON_GetObjectItem(root, "optimize")) : false;

    std::stringstream cmd;
#ifdef _WIN32
    cmd << "window_build\\calib.exe";
#else
    cmd << "./calib";
#endif
    cmd << " " << inDir << " " << outDir;

    if (doRadio) {
        cmd << " --radio";
        if (!radioRefFile.empty()) cmd << " --ref " << radioRefFile;
    }

    if (autoRadioThickness >= 0) {
        cmd << " --auto";
        if (autoRadioThickness > 0) cmd << " " << autoRadioThickness;
    }

    if (!radioTemplatePath.empty()) cmd << " --template " << radioTemplatePath;
    if (optimize) cmd << " --optimize";

    cmd << " 2>&1"; // Capture stderr

    std::string full_cmd = cmd.str();
    printf("Executing: %s\n", full_cmd.c_str());

    // Remove output dir first
#ifdef _WIN32
    std::string rm_cmd = "rmdir /s /q " + outDir;
#else
    std::string rm_cmd = "rm -rf " + outDir;
#endif
    system(rm_cmd.c_str());

    FILE* pipe = popen(full_cmd.c_str(), "r");
    std::string output;
    char buffer[128];
    if (pipe) {
        while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
            output += buffer;
        }
    }
    int exit_code = pclose(pipe);

    cJSON *res_root = cJSON_CreateObject();
    cJSON_AddNumberToObject(res_root, "code", exit_code);
    cJSON_AddStringToObject(res_root, "stdout", output.c_str());
    cJSON_AddStringToObject(res_root, "stderr", ""); 
    cJSON_AddStringToObject(res_root, "command", full_cmd.c_str());

    char *json_res = cJSON_Print(res_root);
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", json_res);

    free(json_res);
    cJSON_Delete(res_root);
    cJSON_Delete(root);
}

static void ev_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        
        if (match_uri(hm->uri, "/run")) {
            handle_run(c, hm);
        } else if (match_uri(hm->uri, "/list")) {
            handle_list(c, hm);
        } else {
            struct mg_http_serve_opts opts = {0};
            // Set Cache-Control for 1 hour (3600 seconds)
            opts.extra_headers = "Cache-Control: public, max-age=3600\r\n";
            
            char path[512];
            mg_snprintf(path, sizeof(path), "%s/%.*s", s_root_dir, (int) hm->uri.len, hm->uri.buf);
            
            struct stat st;
            if (stat(path, &st) == 0) {
                opts.root_dir = s_root_dir;
                mg_http_serve_dir(c, hm, &opts);
            } else {
                // If serving from current directory, we still apply the cache header
                opts.root_dir = ".";
                mg_http_serve_dir(c, hm, &opts);
            }
        }
    }
}

int main() {
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);
    
    std::string port = "11918";
    char *env_port = getenv("PORT");
    if (env_port != NULL) {
        port = env_port;
        printf("Using port from environment: %s\n", port.c_str());
    }
    
    std::string listen_url = "http://0.0.0.0:" + port;
    
    if (mg_http_listen(&mgr, listen_url.c_str(), ev_handler, NULL) == NULL) {
        printf("Error starting server on %s\n", listen_url.c_str());
        return 1;
    }
    printf("Server started on http://localhost:%s/webui/\n", port.c_str());
    for (;;) mg_mgr_poll(&mgr, 1000);
    mg_mgr_free(&mgr);
    return 0;
}
