import React from "react";
import classNames from "classnames";
import intl from 'react-intl-universal';

export default class DrivingMode extends React.PureComponent {
    constructor(props) {
        super(props);

        this.utterance = window.speechSynthesis ? new SpeechSynthesisUtterance() : null;
    }

    componentWillUpdate() {
        if (this.utterance) {
            window.speechSynthesis.cancel();
        }
    }

    render() {
        const { drivingMode, isAutoMode } = this.props;

        if (this.utterance) {
            this.utterance.text = `Entering to ${drivingMode} mode`;
            window.speechSynthesis.speak(this.utterance);
        }

        const drivingModeTip = intl.get(drivingMode);
        return (
            <div className={classNames({
                        "driving-mode": true,
                        "auto-mode": isAutoMode,
                        "manual-mode": !isAutoMode,
                    })}>
                <span className="text">{drivingModeTip}</span>
            </div>
        );
    }
}
