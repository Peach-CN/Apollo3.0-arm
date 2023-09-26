import React from "react";
import classNames from "classnames";

export default class ProgressComponent extends React.Component {
    render() {
        return (
            <div className={this.props.done ? 'progress-item done' : 'progress-item'} style={{ width: this.props.width ? this.props.width + 'px' : '' }}>
                <div className="progress-title">{this.props.title}</div>
                <div className="progress-body">
                    <h3>{this.props.number}</h3>
                    <div className="progress-line"></div>
                </div>
            </div>
        );
    }
}