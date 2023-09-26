import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import WS from "store/websocket";

import intl from 'react-intl-universal';

class CommandGroup extends React.Component {
    render() {
        const { name,title, commands,disabled,
                extraCommandClass, extraButtonClass} = this.props;

        const entries = Object.keys(commands).map((key) => {
            return <button className={extraButtonClass}
                           disabled={disabled}
                           key={key}
                           onClick={commands[key]}>{title}</button>;
        });

        const text = name ? <span className="name">{`${name}:`}</span> : null;

        return (
            <div className={classNames("command-group", extraCommandClass)}>
                {text}
                {entries}
            </div>
        );
    }
}

@inject("store") @observer
export default class QuickStarter extends React.Component {

    constructor(props) {
        super(props);

        this.utterance = window.speechSynthesis ? new SpeechSynthesisUtterance() : null;

        this.rtKRecord = {
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_recorder");
                this.speechSynthesis('Start RTK recorder');
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_recorder");
                this.speechSynthesis('Stop RTK recorder');
            },
        };

        this.rtkReplay = {
            "Start": () => {
                WS.executeToolCommand("rtk_record_replay", "start_player");
                this.speechSynthesis('Start RTK replay');
            },
            "Stop": () => {
                WS.executeToolCommand("rtk_record_replay", "stop_player");
                this.speechSynthesis('Stop RTK replay');
            },
        };

        this.setup = {
            "Setup": () => {
                WS.executeModeCommand("start");
                this.speechSynthesis('Setup');
            },
        };

        this.reset = {
            "Reset All": () => {
                WS.executeModeCommand("stop");
                this.speechSynthesis('Reset All');
            },
        };

        this.auto = {
            "Start Auto": () => {
                WS.changeDrivingMode("COMPLETE_AUTO_DRIVE");//发送自动驾驶指令
                this.speechSynthesis('Start Auto');
                console.log("开始自动驾驶");
            },
        };
        this.manual = {
            "Start Manual": () => {
                WS.changeDrivingMode("COMPLETE_MANUAL");//发送手动驾驶指令
                this.speechSynthesis('Start Manual');
                console.log("开始手动驾驶");
            },
        };
    }

    componentWillUpdate() {
        if (this.utterance) {
            window.speechSynthesis.cancel();
        }
    }

    speechSynthesis(phrase) {
        if (this.utterance) {
            this.utterance.text = phrase;
            window.speechSynthesis.speak(this.utterance);
        }
    }

    render() {
        const { hmi } = this.props.store;
        const { tasksPanelLocked } = this.props.store.options;

        return (
            <div className="card">
                <div className="card-header">
                    <span>{intl.get('Quick_Start')}</span>
                </div>
                <div className="card-content-column">
                    <CommandGroup title={intl.get('Setup')} disabled={tasksPanelLocked} commands={this.setup} />
                    <CommandGroup title={intl.get('Reset_All')} disabled={tasksPanelLocked} commands={this.reset} />
                    <CommandGroup title={intl.get('Start_Auto')} disabled={!hmi.enableStartAuto || tasksPanelLocked}
                                  commands={this.auto}
                                  extraButtonClass="start-auto-button"
                                  extraCommandClass="start-auto-command" />
                    <CommandGroup title={intl.get('Start Manual')} disabled={hmi.enableStartAuto || tasksPanelLocked}
                                  commands={this.manual}
                                  extraButtonClass="start-auto-button"
                                  extraCommandClass="start-auto-command" />
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Record"
                                      disabled={tasksPanelLocked}
                                      commands={this.rtKRecord} />}
                    {hmi.showRTKCommands &&
                        <CommandGroup name="Replay"
                                      disabled={tasksPanelLocked}
                                      commands={this.rtkReplay} />}
                </div>
            </div>
        );
    }
}