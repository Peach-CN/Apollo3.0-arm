import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";

import { millisecondsToTime } from "utils/misc";

import intl from 'react-intl-universal';


class Delay extends React.PureComponent {
    render() {
        const { time, warning } = this.props;

        const timeString = (time === '-') ? time : millisecondsToTime(time | 0);

        return (
            <div className={classNames({"value": true, "warning": warning})}>
                {timeString}
            </div>
        );
    }
}


@inject("store") @observer
export default class DelayTable extends React.Component {
    render() {
        const { moduleDelay } = this.props.store;

        const items = moduleDelay.keys().sort()
            .map(key => {
                const module = moduleDelay.get(key);
                const warning = module.delay > 2000 && module.name !== "TrafficLight";
                const name = intl.get(module.name);//获取对应模块的翻译
                return (
                    <div className="delay-item" key={'delay_' + key}>
                        <div className="name">{name}</div>
                        <Delay time={module.delay} warning={warning} />
                    </div>
                );
            });

        return (
            <div className="delay card">
                <div className="card-header"><span>{ intl.get('Module_Delay') }</span></div>
                <div className="card-content-column">
                    {items}
                </div>
            </div>
        );
    }
}