#pragma once

#include <future>
#include <utility>

#include <cpprest/uri.h>
#include <cpprest/http_listener.h>

using namespace std;
using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;


namespace rest_api {

using status_codes = web::http::status_codes;

/**
 * @brief helper class to provide the correct response to the restcpp callbacks
 * 
 */
struct Response : public web::http::http_response {
    explicit Response(const http::status_code &code) : http_response(code) {}
    Response(const http::status_code &code, const std::string& body, const std::string& content_type = "text/plain; charset=utf-8") : http_response(code) {
        this->set_body(body, utility::conversions::to_string_t(content_type));
    }
    Response(const http::status_code &code, const json::value &body):http_response(code){
        this->set_body(body);
    }
};

/**
 * @brief Server class implementation that can serve files in a folder and 
 * that allows registering GET or POST URLs to forward to a std::function,
 * which includes lambda functions
 */

class RestServer {
    public:
        /**
         * @brief map thet holds the parts of the get query part
         */
        typedef std::map<utility::string_t, utility::string_t> GetQuery;

        /**
         * @brief callback function for GET requests
         * e.g. http://127.0.0.1:34568/api/mycallback?value=1&value2=2 
         * translated to getQuery["value1"]=1;getQuery["value2"]=2;
         */
        typedef std::function< web::http::http_response (GetQuery, http_request& message)> GetCallback;

        /**
         * @brief callback function for POST requests
         * to access data in the callback use e.g. message.extract_json().get();
         */
        typedef std::function< web::http::http_response (http_request& message)> PostCallback;


        /**
         * @brief Construct a new Rest Server object
         * 
         * @param url the url to serve, e.g. "http://127.0.0.1:34568"
         */
        RestServer(const std::string& url);
        virtual ~RestServer() {}

        void allowCORS(const std::string& origin = "*") {
            addToOptionHeader("Access-Control-Allow-Origin", origin);
            addToResponseHeader("Access-Control-Allow-Origin", origin);
        }

        /**
         * @brief start the server
         */
        void startListen();

        /**
         * @brief stop the server
         * 
         */
        void stopListen();

        /**
         * @brief set a folder that will be served under the html url/
         * 
         * @param folderpath the local folder to serve
         * @param url the url part the folder content should be available. "API" is reserved
         */
        void serveFolder(const std::string& folderpath, const std::string& url = "html") {
            servedFolders[url] = folderpath;
        }

        /**
         * @brief Set the Default Page when the server is accessed, is unset the default listing (api/ html/ will be shown)
         * 
         * @param localurl 
         */
        void setDefaultPage(const std::string& localurl){
            defaultpage = localurl;
        };

        /**
         * @brief register a GET url for api calls
         * 
         * @param localuri the url to make the registered call available on
         * @param cb the callback to ececute when the url is called
         */
        void registerGetCallback(const std::string& localuri, const GetCallback& cb, const std::string documentation = "", const std::string sample = "") {
            getCallbacks["/api/"+localuri] = cb;
            if (documentation != "") { 
                docStrings["/api/"+localuri] = documentation;
            }
            if (sample != "") {
                sampleStrings["/api/"+localuri] = sample;
            }
        }

        /**
         * @brief register a POST url for api calls
         * 
         * @param localuri the url to make the registered call available on
         * @param cb the callback to ececute when the url is called
         */
        void registerPostCallback(const std::string& localuri, const PostCallback& cb, const std::string documentation = "", const std::string sample = "") {
            postCallbacks["/api/"+localuri] = cb;
            if (documentation != "") { 
                docStrings["/api/"+localuri] = documentation;
            }
            if (sample != "") {
                sampleStrings["/api/"+localuri] = sample;
            }
        }

        void addToResponseHeader(const std::string& key, const std::string& value) {
            additionalResonseHeaders.push_back(std::pair<std::string, std::string>(key, value));
        }
        void removeFromResponseHeader(const std::string& key) {
            additionalResonseHeaders.erase(std::remove_if(additionalResonseHeaders.begin(), additionalResonseHeaders.end(), [&key](const std::pair<std::string, std::string>& entry){
                return entry.first == key;
            }));
        }

        void addToOptionHeader(const std::string& key, const std::string& value) {
            additionalOptionHeaders.push_back(std::pair<std::string, std::string>(key, value));
        }
        void removeFromOptionHeader(const std::string& key) {
            additionalOptionHeaders.erase(std::remove_if(additionalOptionHeaders.begin(), additionalOptionHeaders.end(), [&key](const std::pair<std::string, std::string>& entry){
                return entry.first == key;
            }));
        }

        pplx::task<void>open() {return m_listener.open();}
        pplx::task<void>close() {return m_listener.close();}

    private:
        /**
         * @brief callback for restcpp to hangle GET messages
         * 
         * @param message 
         */
        void handle_get(http_request message);

        /**
         * @brief callback for restcpp to hangle POST messages
         * 
         * @param message 
         */
        void handle_post(http_request message);

        /**
         * @brief callback for restcpp to hangle OPTIONS messages
         * 
         * @param message 
         */
        void handle_options(http_request message);

        /**
         * @brief message reply function that adds custom headers to the message
         * Headers can be set by addToResponseHeader() or allowCORS() function
         * 
         * @param message 
         */
        void reply_with_headers(http_request *message, http_response *response);

        /**
         * @brief internal function to send files of folders gegistered through serveFolder()
         * 
         * @param path the path of the called url split into a vector removign the "/"
         * @param alreadyEvaluated number of elements of the path already evaluated
         * @param message the original request
         */
        void serve_folder(const std::vector<utility::string_t> &path, int alreadyEvaluated, http_request& message);

        /**
         * @brief internal function to call the callbacks registered by registerGetCallback() or registerPostCallback()
         * 
         * @param path the path of the called url split into a vector removign the "/"
         * @param alreadyEvaluated number of elements of the path already evaluated
         * @param message the original request
         */
        void serve_api(const std::vector<utility::string_t> &path, int alreadyEvaluated, http_request& message);


        /**
         * @brief the restcpp listener
         */
        http_listener m_listener;

        /**
         * @brief mappings, url part to file or api callback
         */
        std::map<std::string, std::string> servedFolders;
        std::map<std::string, GetCallback> getCallbacks;
        std::map<std::string, PostCallback> postCallbacks;
        std::map<std::string, std::string> docStrings;
        std::map<std::string, std::string> sampleStrings;

        std::string defaultpage;

        std::vector< std::pair<std::string, std::string> > additionalResonseHeaders;
        std::vector< std::pair<std::string, std::string> > additionalOptionHeaders;
};

} // end namespace rest_api


