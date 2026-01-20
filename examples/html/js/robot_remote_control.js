

class RobotController {

    constructor(ip, commandport, telemetryport) {
        this.host = ip
        this.commandport = commandport
        this.telemetryport = telemetryport

        this.dependentsimpleactionforms = new Array();

        this.connectTelemetry()
        this.connectCommands()

    }

    close() {
        this.telemetryws.close();
    }
    
    get(url, callback) {
        var xmlHttp = new XMLHttpRequest()
        xmlHttp.open( "GET", url, false ) // false for synchronous request
        xmlHttp.send( null )
        callback(JSON.parse(xmlHttp.responseText))
    }

    post(url, data, callback){
        var xmlHttp = new XMLHttpRequest()
        xmlHttp.open( "post", url, false ) // false for synchronous request
        xmlHttp.setRequestHeader('Content-type', 'application/json');
        xmlHttp.send( JSON.stringify(data) )
        callback(JSON.parse(xmlHttp.responseText))
    }

    connectCommands() {
        this.updateSimpleActions();
    }

    connectTelemetry() {
        this.telemetryws = new WebSocket("ws://"+host+":"+telemetryport);
        this.telemetryws.onopen=function(evt) {
            console.log("opened");

            //todo enable heartbeat
            //todo request simple actions

        }
        this.telemetryws.onmessage=function(evt) {
            // parse wrapping TelemetryMessage js Object

            // if (evt.data != "") {
                var telemetryMessage = JSON.parse(evt.data)

                
                // parse actual data into js Object
                var json = JSON.parse(telemetryMessage.json)
                
                // handle different messages
                if (telemetryMessage.type == "CURRENT_POSE") {
                    document.getElementById("pose").innerHTML = telemetryMessage.json
                }
            // }


        };

        this.telemetryws.onclose=function(evt) {
            console.log("closed");
        };
    }


    generateControlMessage(type, payload) {
        var msg = new Object
        msg["type"] = type
        if (payload!=undefined) {
            msg["json"] = JSON.stringify(payload)
        }
        return msg
    }
    generateTelemetryRequest(type, channel) {
        var msg = new Object
        msg["type"] = type
        if (channel!=undefined) {
            msg["channel"] = channel
        }
        return msg
    }


    setSimpleAction(name, value){
        var simpleactioncommand = new Object;
        simpleactioncommand["name"] = name;
        simpleactioncommand["state"] = value;
        this.post("http://"+host+":"+commandport+"/api/command", this.generateControlMessage("SIMPLE_ACTIONS_COMMAND", simpleactioncommand),function (reply){
            console.log(reply)
        })
    }



    setValue(id, value) {
        document.getElementById(id).value = value;
    }

    setDependentSimpleActions(newactions){
        this.dependentsimpleactionforms = newactions;
        
        //loop actions and add update to click on the dependent ones
        this.dependentsimpleactionforms.forEach(function(action){
            // console.log(action);
            // console.log(action.depend.dependsOnAction);
            actionForm = document.getElementById(action.depend.dependsOnAction);
            command =  'updateDependentSimpleActions();'
            actionForm.setAttribute("onclick", command);
        });
        //update current state on forms
        this.updateDependentSimpleActions();
    }

    updateDependentSimpleActions() {
        console.log("updateDependentSimpleActions");
        //loop 
        this.dependentsimpleactionforms.forEach(function(action){
            //get dependent action
            var parentaction = document.getElementById(action.depend.dependsOnAction);
            
            // get current form value
            var formvalue = document.querySelector('input[name="'+action.depend.dependsOnAction+'"]:checked').value;


            var formdiv = document.getElementById(action.name+"div");

            if (action.depend.dependsOnActionInState == formvalue) {
                if (!formdiv.hasChildNodes()) {
                    //re-add
                    formdiv.appendChild(action.form);
                }
            }else{
                if (formdiv.hasChildNodes()) {
                    formdiv.removeChild(action.form);
                }
            }
        });
    }

    updateSimpleActions() {
        var self = this // this var to call functions from lambda functions (this whould be the lamda)
        this.get("http://"+host+":"+commandport+"/api/simpleActions", function (data) {
            console.log(data)
            console.log(data.actions)
            var actionselement = document.getElementById("simpleactions");
            actionselement.innerHTML = "";

            var dependentactions = new Array();

            data.actions.forEach( function(simpleaction) {
                var formdiv = document.createElement("div");
                formdiv.setAttribute("id", simpleaction.name+"div");
                var form = document.createElement("form");
                form.setAttribute("id", simpleaction.name);
                formdiv.appendChild(form);
                

                if (simpleaction.type.type == "TRIGGER") {
                    var button = document.createElement("button");
                    button.setAttribute("type", "button");
                    var command = 'rrc.setSimpleAction("'+ simpleaction.name + '",1);'
                    button.setAttribute("onclick", command);
                    button.setAttribute("id", simpleaction.name+"_button");
                    button.setAttribute("class", "simpleaction_input_button");
                    button.innerHTML = simpleaction.name
                    form.appendChild(button);
                    //actionselement.appendChild(document.createElement("br"));
                } else if (simpleaction.type.valueNames !== undefined) {
                    // has named values
                    //these need to be added/removed on selection of another action
                    if (simpleaction.type.actionDependency !== undefined) {
                        var action = new Object();
                        action["name"] = simpleaction.name;
                        action["depend"] = simpleaction.type.actionDependency;
                        action["form"] = form;
                        dependentactions.push(action);
                    }
                    field = document.createElement("fieldset");
                    field.name = simpleaction.name;
                    field.innerHTML = simpleaction.name;
                    field.appendChild(document.createElement("br"));
                    simpleaction.type.valueNames.forEach( function(namedValue) {
                        input = document.createElement("input");
                        input.setAttribute("type", "radio");
                        input.setAttribute("name", simpleaction.name);
                        input.setAttribute("id", namedValue.name);
                        //protobuf leaves out default values, so undefined values are equal to the default value
                        var value = 0;
                        if (namedValue.value !== undefined) {
                            value = namedValue.value;
                        }
                        var state = 0;
                        if (simpleaction.state !== undefined) {
                            state = simpleaction.state;
                        }
                        if (value == state ) {
                            input.setAttribute("checked","checked");
                        }
                        input.setAttribute("value", value);
                        label = document.createElement("label");
                        label.setAttribute("for", namedValue.name);
                        label.innerHTML = " " + namedValue.name;

                        command =  'rrc.setSimpleAction("'+ simpleaction.name + '",this.value);'                            
                        input.setAttribute("onchange", command);

                        field.appendChild(input);
                        field.appendChild(label);
                        field.appendChild(document.createElement("br"));
                    });
                    form.appendChild(field);
                } else {
                    var input = document.createElement("input");
                    var slider = document.createElement("input");
                    //inputname = document.createElement("div");
                    form.innerHTML = simpleaction.name;
                    input.setAttribute("type", "number");
                    slider.setAttribute("type", "range");
                    input.setAttribute("id", "simpelactioninput" + simpleaction.name);
                    slider.setAttribute("id", "simpelactionslider" + simpleaction.name);
                    input.setAttribute("value", simpleaction.state);
                    if (simpleaction.type.maxState !== undefined) {
                        input.setAttribute("max", simpleaction.type.maxState);
                        slider.setAttribute("max", simpleaction.type.maxState);
                    }
                    if (simpleaction.type.minState !== undefined) {
                        input.setAttribute("min", simpleaction.type.minState);
                        slider.setAttribute("min", simpleaction.type.minState);
                    }else{
                        input.setAttribute("min", 0);
                        slider.setAttribute("min", 0);
                    }
                    if (simpleaction.type.stepSize !== undefined) {
                        input.setAttribute("step", simpleaction.type.stepSize);
                        slider.setAttribute("step", simpleaction.type.stepSize);
                    }
                    
                    input.setAttribute("value", simpleaction.state);
                    slider.setAttribute("value", simpleaction.state);
                    command =  'rrc.setSimpleAction("'+ simpleaction.name + '",this.value);'
                    var inputupdate = 'rrc.setValue("simpelactioninput'+ simpleaction.name +'",this.value);'
                    var sliderupdate = 'rrc.setValue("simpelactionslider'+ simpleaction.name +'",this.value);'
                    input.setAttribute("onchange", command + sliderupdate);
                    slider.setAttribute("onchange", command + inputupdate);

                    input.setAttribute("style", "width: 45px");

                    //form.appendChild(inputname);
                    form.appendChild(slider);
                    form.appendChild(input);
                    
                    //actionselement.appendChild(document.createElement("br"));
                }
                actionselement.appendChild(formdiv);
            });
            self.setDependentSimpleActions(dependentactions);
        });
    }




}