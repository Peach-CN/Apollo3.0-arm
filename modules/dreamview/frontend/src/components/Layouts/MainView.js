import React from "react";
import { inject, observer } from "mobx-react";
import Loadable from 'react-loadable';
import classNames from "classnames";
import RouteEditingBar from "components/RouteEditingBar";
import StatusBar from "components/StatusBar";
import Scene from "components/Scene";
import Loader from "components/common/Loader";
import PlaybackControls from "components/PlaybackControls";
import ReactTooltip from 'react-tooltip';
import demoIcon1 from "assets/images/icons/0408K5.png";
import demoIcon2 from "assets/images/icons/0408K6.png";
import demoIcon3 from "assets/images/icons/0408K7.png";
import intl from 'react-intl-universal';


const Navigation = Loadable({
    loader: () => import("components/Navigation"),
    loading() {
      return <div>Loading...</div>;
    }
});


class SensorCamera extends React.Component {
    render() {
        return (
            <div className="video">
                <img src='/image' />
            </div>
        );
    }
}

class TipButton extends React.PureComponent {
    render() {
        const { disabled, onClick, active, label, extraClasses, iconSrc, buttontext } = this.props;
        return (
            <button onClick={onClick}
                disabled={disabled}
                className={classNames({
                    "button": true,
                }, extraClasses)} >
                {buttontext}
            </button>
        );
    }
}

class DemoWizardTips extends React.Component {
    constructor(props) {
        super(props);
        this.handleFinishButton = this.handleFinishButton.bind(this);
        this.handlePrevButton = this.handlePrevButton.bind(this);
        this.handleNextButton = this.handleNextButton.bind(this);
        this.state = {
            index: 0,
            tips: [],
        };
    }
    handleFinishButton() {
        const { handleCloseBtnClick } = this.props;
        handleCloseBtnClick();
    }
    handlePrevButton() {
        const i = this.state.index - 1;
        this.setState({ index: i });
    }
    handleNextButton() {
        const i = this.state.index + 1;
        this.setState({ index: i });
    }
    render() {
        const { handleCloseBtnClick, tips } = this.props;
        if (this.state.tips === tips) {
            //
        } else {
            this.state.index = 0;
            this.state.tips = tips;
        }
        const Tip = intl.get("Tip");
        const Finish = intl.get("Finish");
        return (
            <div className="demo-wizard-tips">
                <div className="title-bar">
                    <span>
                        <button onClick={handleCloseBtnClick} className="close-button">
                            X
                        </button>
                    </span>
                </div>
                <div className="tip-bar">
                    <span>{Tip}</span>
                </div>
                <div className="tip-content">
                    <span>{tips[this.state.index]}</span>
                </div>
                <div className="action-bar">
                    <span>
                        {this.state.index === tips.length - 1 && <TipButton onClick={this.handleFinishButton} buttontext={Finish} extraClasses="submit-button" />}
                        {(this.state.index > 0 && this.state.index < tips.length - 1) && <TipButton onClick={this.handlePrevButton} buttontext="<" extraClasses="submit-button" />}
                        {(this.state.index < tips.length - 1) && <TipButton onClick={this.handleNextButton} buttontext=">" extraClasses="submit-button" />}
                    </span>
                </div>
            </div>
        );
    }
}

class DemonstrationWizard extends React.Component {
    constructor(props) {
        super(props);
        this.handleClickClose = this.handleClickClose.bind(this);
        this.handleButton1Clicked = this.handleButton1Clicked.bind(this);
        this.handleButton2Clicked = this.handleButton2Clicked.bind(this);
        this.handleButton3Clicked = this.handleButton3Clicked.bind(this);
        this.state = {
            visible: false,
            index: 0,
            tips: [],
        };
    }
    handleClickClose() {
        this.setState({ visible: false, index: 0, tips: [] });
    }
    handleButton1Clicked() {
        const tipInfo = intl.get("TRACK_DEMO");
        this.setState({ visible: true, index: 1, tips: tipInfo });
    }
    handleButton2Clicked() {
        const tipInfo = intl.get("ObstacleAvoidance");
        this.setState({ visible: true, index: 2, tips: tipInfo });
    }
    handleButton3Clicked() {
        const tipInfo = intl.get("BypassingObstacles");
        this.setState({ visible: true, index: 3, tips: tipInfo });
    }
    render() {
        let y_offset = '0px';
        if (this.state.index === 1) {
            y_offset = '0px';
        } else if (this.state.index === 2) {
            y_offset = '106px';
        } else if (this.state.index === 3) {
            y_offset = '214px';
        }
        const divStyle = {
            position: 'absolute',
            top: y_offset,
            left: '124px',
            display: 'inline',
            width: '400px',
            height: '160px',
            borderRadius: '2px',
        };
        return (
            <div className="demo-wizard">
                <div className="menu">
                    <div className="wizard">
                        <span>
                            <button className="submit-button" onClick={this.handleButton1Clicked}>
                                <img src={demoIcon1} data-tip={intl.get("TRACK_DEMO_TIP")} /><br />
                                <ReactTooltip place='right' type='light' />
                            </button>
                        </span>
                    </div>
                    <div className="placeholder" />
                    <div className="wizard">
                        <span>
                            <button className="submit-button" onClick={this.handleButton2Clicked}>
                                <img src={demoIcon2} data-tip={intl.get("ObstacleAvoidance_TIP")} /><br />
                            </button>
                        </span>
                    </div>
                    <div className="placeholder" />
                    <div className="wizard">
                        <span>
                            <button className="submit-button" onClick={this.handleButton3Clicked}>
                                <img src={demoIcon3} data-tip={intl.get("BypassingObstacles_TIP")} /><br />
                            </button>
                        </span>
                    </div>
                </div>
                <div className="tips" style={divStyle}>
                    {this.state.visible === true && <DemoWizardTips handleCloseBtnClick={this.handleClickClose} tips={this.state.tips} />}
                </div>
            </div>
        );
    }
}


@inject("store") @observer
class SceneView extends React.Component {
    render() {
        const { sceneDimension, meters, monitor,
            options, trafficSignal, video, hmi } = this.props.store;

        const sceneHeightOffset = OFFLINE_PLAYBACK ? 40 /* height of playback control */ : 0;
        return (
            <div className="main-view" style={{ height: sceneDimension.height }}>
                <Scene width={sceneDimension.width}
                    height={sceneDimension.height - sceneHeightOffset}
                    options={options}
                    invisible={false} />
                {options.showRouteEditingBar
                    ? <RouteEditingBar />
                    : <StatusBar meters={meters}
                        trafficSignal={trafficSignal}
                        showNotification={!options.showTasks}
                        monitor={monitor} />}
                {options.showVideo && <SensorCamera />}
                <DemonstrationWizard />
                {OFFLINE_PLAYBACK && <PlaybackControls />}
                {hmi.inNavigationMode &&
                    <Navigation viewHeight={sceneDimension.height}
                        viewWidth={sceneDimension.width} />}
            </div>
        );
    }
}

@inject("store") @observer
export default class MainView extends React.Component {
    render() {
        const { isInitialized, sceneDimension } = this.props.store;

        if (!isInitialized && !OFFLINE_PLAYBACK) {
            return <Loader height={sceneDimension.height} />;
        } else {
            return <SceneView />;
        }
    }
}

