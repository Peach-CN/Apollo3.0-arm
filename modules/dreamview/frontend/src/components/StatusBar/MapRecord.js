import React from "react";
import PropTypes from 'prop-types';
import { inject, observer } from "mobx-react";
import WS from "store/websocket";

import intl from 'react-intl-universal';
import { Modal, Button } from 'antd';

@inject("store") @observer
export default class MapRecord extends React.Component {
    constructor(props) {
      super(props);
      this.state = {
          mode: 0,
          mapName: "新建地图",
          mapNameTip: "",
          modalAddInfoVisible: false //新增信息Modal的显示属性
      };
      this.recordMap = this.recordMap.bind(this);
    }
    recordMap(){
        console.log("click record button");
        if (this.state.mode === 0) {
            this.setState({modalAddInfoVisible: true});//弹出对话框编辑地图名称
        } else {
            this.setState({
                mode: 0,
            });
            //取消地图录制
            WS.submitRecordRequest("","stop");
        }
    }
    render() {
        const {
            maps
        } = this.props.store.hmi;
        // const maps = ["123","456","789"];
      const name = this.state.mode === 0 ? "Start Record Map" : "Stop Record Map";
      const tip = intl.get(name);
      return (
            <div className="map-record">
                <Modal
                    title="设置地图名称"
                    visible={this.state.modalAddInfoVisible}
                    onOk={() => {
                        this.setState({
                            mode: 1,
                        });
                        this.setState({modalAddInfoVisible: false});
                        console.log(this.state.mapName);
                        //开始地图录制this.state.mapName
                        WS.submitRecordRequest(this.state.mapName,"start");
                    }}
                    onCancel={() => {
                        this.setState({modalAddInfoVisible: false});
                    }}>
                    <span>地图名称:</span>
                    <input value={this.state.mapName} onChange={(e) => {
                        console.log(e.target.value);
                        const len=maps.length;
                        let hasMapName = "no";
                        for(let j = 0; j < len; j++) {
                            if(maps[j] === e.target.value){
                                hasMapName = "yes";
                            }
                        }
                        if(hasMapName === "yes"){
                            this.setState({
                                mapName: e.target.value,
                                mapNameTip: "已存在该地图名称,点击确定覆盖原来地图"
                            });
                        } else {
                            this.setState({
                                mapName: e.target.value,
                                mapNameTip: ""
                            });
                        }
                    }}/>
                    <p style={{color:'red'}}>{this.state.mapNameTip}</p>
                </Modal>
              <button onClick={this.recordMap}>
                    {tip}
              </button>
            </div>
      );
    }
}