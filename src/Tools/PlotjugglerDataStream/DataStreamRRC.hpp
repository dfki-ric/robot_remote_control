#pragma once

#include <QtPlugin>
#include <QTreeWidgetItem>
#include <QLabel>

#include <thread>
#include <string>
#include <utility>
#include <memory>
#include <map>
#include <PlotJuggler/datastreamer_base.h>

#include "../../RobotController/RobotController.hpp"

class DataStreamRRC : public PJ::DataStreamer {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.RRC")
    Q_INTERFACES(PJ::DataStreamer)

 public:

    struct UpdateBool {
        UpdateBool():state(false){}

        bool operator=(const bool &update) {
            if (update) {
                state = true;
            }
            return state;
        }
        bool needsUpdate(){
            return state;
        }
        void reset() {
            state = false;
        }

        bool state;
    };


    DataStreamRRC();


    bool start(QStringList*) override;

    void shutdown() override;

    bool isRunning() const override;

    ~DataStreamRRC() override;

    const char* name() const override {
        return "RRC Streamer";
    }

    bool isDebugPlugin() override {
        return false;
    }

  /// Override this method to save the status of the plugin to XML
  virtual bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const;

  /// Override this method to load the status of the plugin from XML
  virtual bool xmlLoadState(const QDomElement& parent_element);

    // std::pair<QAction*, int> notificationAction() override {
    //     return { _dummy_notification, _notifications_count };
    // }

    /** gereric version for "flat, top level data" might be slower */
    template <class PROTO> void addToPlot(const PROTO& proto, const double stamp, const std::string prefix = "") {
        // printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        const auto reflection = proto.GetReflection();
        std::vector<const  google::protobuf::FieldDescriptor*> fields;
        reflection->ListFields(proto, &fields);
        
        std::deque<const google::protobuf::FieldDescriptor*> queue (fields.begin(), fields.end());

        for (const auto& field : fields) {
        // while (queue.size()) {
        //     const google::protobuf::FieldDescriptor* field = queue.front();
        //     queue.pop_front();
            std::string fieldname = prefix + field->name() + "/";

            if (field->is_repeated()) {
                size_t size = reflection->FieldSize(proto, field);
                for (size_t i = 0; i<size; ++i){

                    if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING){
                        auto& plot = dataMap().getOrCreateStringSeries(fieldname + std::to_string(i));
                        plot.pushBack(PJ::StringSeries::Point(stamp, reflection->GetRepeatedString(proto,field,i))); break;
                    } else {
                        auto& plot = dataMap().getOrCreateNumeric(fieldname + std::to_string(i));    
                        switch (field->cpp_type()) {
                            case google::protobuf::FieldDescriptor::CPPTYPE_INT32: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedInt32(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_INT64: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedInt64(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedUInt32(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedUInt64(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedDouble(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedFloat(proto,field,i))); break;
                            case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetRepeatedBool(proto,field,i))); break;
                        }
                    }
                }
            }else{
                if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING){
                    auto& plot = dataMap().getOrCreateStringSeries(fieldname);
                    plot.pushBack(PJ::StringSeries::Point(stamp, reflection->GetString(proto,field))); break;
                } else {
                    auto& plot = dataMap().getOrCreateNumeric(fieldname);
                    switch (field->cpp_type()) {
                        case google::protobuf::FieldDescriptor::CPPTYPE_INT32: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetInt32(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_INT64: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetInt64(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetUInt32(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetUInt64(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetDouble(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetFloat(proto,field))); break;
                        case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: plot.pushBack(PJ::PlotData::Point(stamp, reflection->GetBool(proto,field))); break;
                    }
                }
            }
        }
    }

    void addToPlot(const robot_remote_control::TimeStamp& ts, const double stamp, const std::string prefix = "");

    void addToPlot(const robot_remote_control::Header& header, const double stamp, const std::string prefix = "");

    void addToPlot(const robot_remote_control::JointState& proto, const double stamp, const std::string prefix = "");


    template<class PROTO> bool handleChannels(const robot_remote_control::TelemetryMessageType &type, const double& stamp) {
        PROTO proto;
        bool newDataReceived = false;
        int channels = robotdata->getMaxChannelNo(type);
        static double headerstamp;
        for (size_t chan = 0; chan<=channels; ++chan) {
            while (robotdata->getTelemetry(type, &proto, false, chan)) {
                headerstamp = proto.header().timestamp().secs() + proto.header().timestamp().nsecs() / 1000000000.0;
                if (!useDataTimestamp || headerstamp == 0) {
                    headerstamp = stamp;
                }
                newDataReceived = true;
                addToPlot(proto, headerstamp, robot_remote_control::TelemetryMessageType_Name(type) + std::to_string(chan) + "/");
            }
            // no new data, but we still want to write the plot
            // addToPlot(proto, stamp, robot_remote_control::TelemetryMessageType_Name(type) + std::to_string(chan) + "/");
            
        }
        return newDataReceived;
    }


 public slots:
    void selectionChanged(QTreeWidgetItem * item, int column);
    void itemClicked(QTreeWidgetItem * item, int column);
    void setUseDataTimestamp(int state);
    void setTimeStampName(const QString &text);

 private:


    std::shared_ptr<robot_remote_control::RobotController> robotdata;
    robot_remote_control::ChannelsDefinition channeldef;

    void loop();

    std::thread _thread;

    bool _running;


    UpdateBool newData;
    void pushSingleCycle();


    bool useDataTimestamp;

    // QLabel* optiontext;

    // std::unique_ptr<QTreeWidget> treewidget;

    // std::map<QTreeWidgetItem*, ConnectionData> conenctions;

    // // portoptions access to use in pushSingleCycle
    // std::map<RTT::base::InputPortInterface*, std::shared_ptr<PortOptions>> portOptions;
    // // option acces to use in selectionChanged slot
    // std::map<QTreeWidgetItem*, std::shared_ptr<PortOptions>> options_use_data_timestamp;
    // std::map<QTreeWidgetItem*, std::shared_ptr<PortOptions>> options_timestamp_field_name;

    // // data loaded in order to initialize
    // // std::map<taskname, std::map<portname , connectonstart> >
    // std::map<std::string, std::map<std::string, bool>> connectOnStartup;
    // std::map<std::string, std::map<std::string, std::shared_ptr<PortOptions>>> inputPortOptionsOnStartup;
};
