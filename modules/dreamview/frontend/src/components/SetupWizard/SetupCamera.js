import React from "react";
import classNames from "classnames";
import ReactSwiper from 'reactjs-swiper';
import stepIcon1 from "assets/images/icons/bicon1.png";
import stepIcon2 from "assets/images/icons/bicon2.png";
import stepIcon3 from "assets/images/icons/bicon3.png";
import stepIcon4 from "assets/images/icons/bicon4.png";
import stepIcon5 from "assets/images/icons/bicon5.png";
import stepIcon6 from "assets/images/icons/bicon6.png";
import stepIcon7 from "assets/images/icons/bicon7.png";

export default class SetupCameraComponent extends React.Component {
	constructor(props) {
		super(props);
        this.onButtonClicked = this.onButtonClicked.bind(this);
    }
    onButtonClicked() {
        const { handleNextButton } = this.props;
        handleNextButton();
    }
    render() {
        const items = [{
            image: stepIcon1,
            title: '相机检测'
        }, {
            image: stepIcon2,
            title: '通讯配置'
        }, {
            image: stepIcon3,
            title: '相机安装'
        }, {
            image: stepIcon4,
            title: '相机学习',
        }, {
            image: stepIcon5,
            title: '相机校正',
        }, {
            image: stepIcon6,
            title: '预警设置',
        }, {
            image: stepIcon7,
            title: '设置完成',
        }];
        const swiperOptions = {
            preloadImages: true,
            autoplay: 4000,
            autoplayDisableOnInteraction: true
        };
        return (
            <div className="page setup-camera-page">
                <div className="page-header">
                    <span>相机校准</span>
                </div>
                <div className="page-content">
                    <div className="image-list">
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon3} />
                            </div>
                            <br />
                            <span>相机检测</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon1} />
                            </div>
                            <br />
                            <span>通讯配置</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon4} />
                            </div>
                            <br />
                            <span>相机安装</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon2} />
                            </div>
                            <br />
                            <span>相机学习</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon6} />
                            </div>
                            <br />
                            <span>相机校正</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon5} />
                            </div>
                            <br />
                            <span>预警设置</span>
                        </div>
                        <div className="connect-line" />
                        <div className="step-item">
                            <div className="step-item-img">
                                <img src={stepIcon7} />
                            </div>
                            <br />
                            <span>同步设置</span>
                        </div>
                    </div>
                    <div className="next-step-button">
                        <span className="calibration-tips">请在Windows上使用相机标定软件完成以上步骤</span>
                        <span>
                            <button onClick={this.onButtonClicked} className="submit-button">
                                <span>下一步</span>
                            </button>
                        </span>
                    </div>
                </div>
            </div>
        );
    }
}