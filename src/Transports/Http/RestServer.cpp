#include "RestServer.hpp"
#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <cpprest/filestream.h>
#include <boost/filesystem.hpp>


//using namespace std;
//using namespace rest_api;


namespace rest_api {

// https://github.com/Meenapintu/Restweb/blob/master/src/handler.cpp
RestServer::RestServer(const std::string& url) {
    uri_builder uri(url);

    auto addr = uri.to_uri().to_string();

    m_listener = http_listener(addr);

    m_listener.support(methods::GET, std::bind(&RestServer::handle_get, this, std::placeholders::_1));
    m_listener.support(methods::POST, std::bind(&RestServer::handle_post, this, std::placeholders::_1));
    m_listener.support(methods::OPTIONS, std::bind(&RestServer::handle_options, this, std::placeholders::_1));

}

void RestServer::startListen() {
    this->open().wait();
}

void RestServer::stopListen() {
    this->close().wait();
}


void RestServer::handle_get(http_request message) {
    // ucout <<  message.to_string() << endl;

    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));

    if (paths.size() == 0) {
        if (defaultpage != "") {
            std::stringstream page;
            page << "<!doctype html><html lang=\"en\">";
            page << "<head>";
            page << "<meta charset=\"utf-8\"><title>Directory listing</title>";
            page << "<meta http-equiv=\"refresh\" content=\"0; url="+ defaultpage +"\" />";
            page << "</head>";
            message.reply(status_codes::OK, page.str(), "text/html");
            return;
        } else {    
            std::stringstream page;
            page << "<!doctype html><html lang=\"en\">";
            page << "<head>";
            page << "<meta charset=\"utf-8\"><title>Directory listing</title>";
            page << "</head>";
            page << "<body>";
            page << "Please use:<br>";
            page << "<a href=/api>api</a><br>";
            for (auto &entry : servedFolders) {
                page << "<a href=/" << entry.first << ">" << entry.first <<"</a><br>";
            }
            page << "</body>";
            message.reply(status_codes::OK, page.str(), "text/html");
            return;
        }
    }

    if (paths[0] == "api") {
        serve_api(paths, 1, message);
        return;
    }

    // todo: change to registered folders
    if  (paths[0] == "html") {
        serve_folder(paths, 1, message);
        return;
    } else {
        //paths.insert(paths.begin(), "html");
        serve_folder(paths, 1, message);
        return;
    }
}

void RestServer::handle_post(http_request message) {
    // ucout <<  message.to_string() << endl;

    auto paths = http::uri::split_path(http::uri::decode(message.relative_uri().path()));

    if (paths.size() == 0) {
        std::stringstream page;
        page << "<!doctype html><html lang=\"en\">";
        page << "<head>";
        page << "<meta charset=\"utf-8\"><title>Directory listing</title>";
        page << "</head>";
        page << "<body>";
        page << "Please use:<br>";
        page << "<a href=/api>api</a><br>";
        for (auto &entry : servedFolders) {
            page << "<a href=/" << entry.first << ">" << entry.first <<"</a><br>";
        }
        page << "</body>";
        message.reply(status_codes::NotFound, page.str(), "text/html");
        return;
    }

    // cout <<  paths[0] << endl;

    if (paths[0] == "api") {
        serve_api(paths, 1, message);
        return;
    }

    return;
}

void RestServer::handle_options(http_request message) {
    printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
    http_response response(status_codes::OK);
    response.headers().add(U("Allow"), U("GET, POST, OPTIONS"));
    response.headers().add(U("Access-Control-Allow-Methods"), U("GET, POST, OPTIONS"));
    response.headers().add(U("Access-Control-Allow-Headers"), U("Content-Type"));
    if (additionalOptionHeaders.size()) {
        std::for_each(additionalOptionHeaders.begin(), additionalOptionHeaders.end(), [&](const std::pair<std::string, std::string> &head) {
            response.headers().add(U(head.first), U(head.second));
        });
    }
    message.reply(response);
}


void RestServer::serve_api(const std::vector<utility::string_t> &path, int alreadyEvaluated, http_request& message) {
    utility::string_t uripath = http::uri::decode(message.relative_uri().path());
    // check if cb is registered
    if (getCallbacks.find(uripath) != getCallbacks.end()) {
        std::map<utility::string_t, utility::string_t> query = http::uri::split_query(http::uri::decode(message.request_uri().query()));
        // std::cout << "GET : " << message.request_uri().to_string() << endl;
        // call the registered callback
        web::http::http_response resonse = getCallbacks[uripath](query, message);
        reply_with_headers(&message, &resonse);
    } else if (postCallbacks.find(uripath) != postCallbacks.end()) {
        // std::map<utility::string_t, utility::string_t> query = http::uri::split_query(http::uri::decode(message.request_uri().query()));

        // call the registered callback
        // std::cout << "POST: " << message.request_uri().to_string() << endl;
        web::http::http_response resonse = postCallbacks[uripath](message);
        reply_with_headers(&message, &resonse);
    } else {
        web::json::value registered;
        // registered["GET"] = web::json::value();
        // registered["POST"] = web::json::value();


        std::map<std::string, rest_api::RestServer::GetCallback>::iterator get = getCallbacks.begin();

        for (unsigned int i = 0; get != getCallbacks.end(); ++i, ++get) {
            web::json::value entry;
            entry["type"] = web::json::value("GET");
            std::string doc = docStrings[get->first];
            if (doc != ""){
                entry["documentation"] = web::json::value(doc);
            }
            std::string sample = sampleStrings[get->first];
            if (sample != ""){
                entry["sample"] = web::json::value(sample);
            }
            registered[get->first] = entry;
        }
        std::map<std::string, rest_api::RestServer::PostCallback>::iterator post = postCallbacks.begin();
        for (unsigned int i = 0; post != postCallbacks.end(); ++i, ++post) {
            web::json::value entry;
            entry["type"] = web::json::value("POST");
            std::string doc = docStrings[post->first];
            if (doc != ""){
                entry["documentation"] = web::json::value(doc);
            }
            std::string sample = sampleStrings[post->first];
            if (sample != ""){
                entry["sample"] = web::json::value::parse(sample);
            }
            registered[post->first] = web::json::value(entry);
        }
        Response resonse(status_codes::NotFound, registered);
        reply_with_headers(&message, &resonse);
    }
}

void RestServer::serve_folder(const std::vector<utility::string_t> &path, int alreadyEvaluated, http_request& message) {
    // cout << message.request_uri().to_string() << endl;

    // map to server side folder
    // get map to local folder
    std::string folder = path[alreadyEvaluated-1];
    std::string localpath = servedFolders[folder];

    // add rest of uri path
    for (unsigned int i = alreadyEvaluated; i < path.size(); ++i) {
        localpath += "/";
        localpath += path[i];
    }

    // append rest of path
    std::string uripath = message.relative_uri().path();
    if (uripath.back() != '/') {
        uripath += '/';
    }

    if (boost::filesystem::is_directory(localpath)) {
        // create directory listing
        std::stringstream page;
        page << "<!doctype html><html lang=\"en\">";
        page << "<head>";
        page << "<meta charset=\"utf-8\"><title>Directory listing</title>";
        page << "</head>";
        page << "<body>";
            boost::filesystem::directory_iterator file(localpath);
            boost::filesystem::directory_iterator end;
            while (file != end) {
                std::string pathname =  file->path().string();
                std::string filename =  file->path().filename().string();
                if (boost::filesystem::is_directory(pathname)) {
                    page << "<a href=\""+uripath+filename+"\">"+filename+"/</a><br>";
                } else {
                    page << "<a href=\""+uripath+filename+"\">"+filename+"</a><br>";
                }
                ++file;
            }
        page << "</body>";
        message.reply(status_codes::OK, page.str(), "text/html");
        return;
    }

    // check
    if (!boost::filesystem::is_regular_file(localpath)) {
        message.reply(status_codes::NotFound, U("404"))
                .then([](pplx::task<void> t){
                    try {
                        t.get();
                    }
                    catch(...){
                        //
                    }
                });
        return;
    }



    concurrency::streams::fstream::open_istream(U(localpath), std::ios::in)
    .then([=](concurrency::streams::istream is){

        // set Mime type, just assume the requested ending is correct
        std::string mime = web::http::details::mime_types::application_octetstream;
        std::string ending = localpath.substr(localpath.find_last_of(".") + 1);
        if ( ending == "html" ) {
            mime = U("text/html");
            // mime = web::http::details::mime_types::text_plain;
        } else if ( ending == "js" ) {
            // mime = web::http::details::mime_types::application_javascript;
            mime = U("text/javascript");
        } else if ( ending == "css" ) {
            mime = U("text/css");
        }

        message.reply(status_codes::OK, is, mime.c_str())
        .then([](pplx::task<void> t){
            try {
                t.get();
            }
            catch(...){
                //
            }
        });
    }).then([=](pplx::task<void>t){
        try {
            t.get();
        }
        catch(...){
            message.reply(status_codes::InternalError, U("INTERNAL ERROR "));
        }
    });
}

void RestServer::reply_with_headers(http_request *message, http_response *response) {
    if (additionalResonseHeaders.size()) {
        std::for_each(additionalResonseHeaders.begin(), additionalResonseHeaders.end(), [&](const std::pair<std::string, std::string> &head) {
            response->headers().add(U(head.first), U(head.second));
        });
    }
    message->reply(*response);
}


}  // namespace rest_api
