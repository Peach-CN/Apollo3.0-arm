import React from "react";
import { ProgressBar, Step } from "react-step-progress-bar";
import classNames from "classnames";
import PropTypes from 'prop-types';
import { observer } from "mobx-react";
import ProgressComponent from 'components/SetupWizard/Progress';
import SetupVehicleComponent from 'components/SetupWizard/SetupVehicle';
import SetupGPSComponent from 'components/SetupWizard/SetupGPS';
import CalibrateChassisComponent from 'components/SetupWizard/CalibrateChassis';
import SetupCameraComponent from 'components/SetupWizard/SetupCamera';

import lineicon1 from "assets/images/icons/lineicon1.png";
import lineicon2 from "assets/images/icons/lineicon2.png";
import SetupRadarComponent from "./SetupRadar";

class WelcomePage extends React.Component {
    render() {
        const { handleNextButton } = this.props;
        return (
            <div className="welcome-page" onClick={handleNextButton}>
                <div className="page-header">
                    <span>欢迎使用低速微型车自动驾驶套件！</span>
                </div>
            </div>
        );
    }
}

class SetupDonePage extends React.Component {
    render() {
        return (
            <div className="setup-done-page">
                <div className="page-header">
                    <span>配置完成！</span>
                </div>
                <div className="page-content">
                    <span>请将计算单元关机并进行整车断电再上电！<br />
                      上电完成后请完成低速微型车自动驾驶套件的功能验证！</span><br />
                </div>
                <div className="finish-button">
                    <span>
                        <button className="submit-button">
                            <span>完成</span>
                        </button>
                    </span>
                </div>
            </div>
        );
    }
}

@observer
export default class SetupWizard extends React.Component {
    constructor(props) {
        super(props);
        this.clickClose = this.clickClose.bind(this);
        this.handlePrevButton = this.handlePrevButton.bind(this);
        this.handleNextButton = this.handleNextButton.bind(this);
        this.state = {
            visible: true,
            index: 0,
            pages: 6,
        };
    }
    clickClose(evt) {
        evt.preventDefault();
        if (this.state.visible) {
            this.setState({ visible: false });
        } else {
            this.setState({ visible: true });
        }
    }
    handlePrevButton() {
        let i = this.state.index;
        if (i > 1) {
            i = i - 1;
        }
        this.setState({ index: i });
    }
    handleNextButton() {
        let i = this.state.index;
        if (i < this.state.pages) {
            i = i + 1;
        }
        this.setState({ index: i });
    }

    render() {
        return (
            <div className="card setup-wizard">
                <div className="center-content">
                    <ProgressBar percent={(this.state.index - 1) * 100 / (this.state.pages - 1)} filledBackground="linear-gradient(to right, #3C50D4, #3C50D4)">
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                        <Step transition="scale">
                            {({ accomplished }) => (
                                <img style={{ filter: `grayscale(${accomplished ? 0 : 80}%)` }} width="24" src={lineicon1} />
                            )}
                        </Step>
                    </ProgressBar>
                    {this.state.index === 0 && <WelcomePage handleNextButton={this.handleNextButton} />}
                    {this.state.index === 1 && <SetupVehicleComponent handleNextButton={this.handleNextButton} />}
                    {this.state.index === 2 && <SetupGPSComponent handleNextButton={this.handleNextButton} />}
                    {this.state.index === 3 && <SetupRadarComponent handleNextButton={this.handleNextButton} />}
                    {this.state.index === 4 && <CalibrateChassisComponent handleNextButton={this.handleNextButton} />}
                    {this.state.index === 5 && <SetupCameraComponent handleNextButton={this.handleNextButton} />}
                    {this.state.index === 6 && <SetupDonePage />}
                </div>
            </div>
        );
    }
}