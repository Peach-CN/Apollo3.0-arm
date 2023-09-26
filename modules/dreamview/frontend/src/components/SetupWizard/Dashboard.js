import React, { Component } from 'react';
import * as echarts from 'echarts';
import 'zrender/lib/svg/svg';

export default class Dashboard extends Component {
    constructor(props) {
        super(props);
        this.state = {
            width: '100%',
            height: '500px'
        };
        this.chart = null;
    }

    async componentDidMount() {
        console.log('did mount');
        await this.initChart(this.el);
        this.setOption(this.props.option);
    }

    componentDidUpdate() {
        this.setOption(this.props.option);
    }

    componentWillUnmount() {
        this.dispose();
    }

    render() {
        const { width, height } = this.state;

        return (
            <div className="default-chart"
                 ref={el => (this.el = el)}
                 style={{ width, height }}
            />
        );
    }

    initChart = el => {
        // renderer 用于配置渲染方式 可以是 svg 或者 canvas
        const renderer = this.props.renderer || 'canvas';
        console.log(renderer);

        return new Promise(resolve => {
            setTimeout(() => {
                this.chart = echarts.init(el, null, {
                    renderer,
                    width: 'auto',
                    height: '400px'
                });
                resolve();
            }, 0);
      });
  };

    setOption = option => {
        if (!this.chart) {
            return;
        }

        const notMerge = this.props.notMerge;
        const lazyUpdate = this.props.lazyUpdate;

        this.chart.setOption(option, notMerge, lazyUpdate);
    };

    dispose = () => {
        if (!this.chart) {
            return;
        }

        this.chart.dispose();
        this.chart = null;
    };

    resize = () => {
        this.chart && this.chart.resize();
    };

    getInstance = () => {
        return this.chart;
    };
}