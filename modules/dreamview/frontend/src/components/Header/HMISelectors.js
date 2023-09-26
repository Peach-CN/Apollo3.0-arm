import React from "react";

import Selector from "components/Header/Selector";
import WS from "store/websocket";

import intl from 'react-intl-universal';

export default class HMISelectors extends React.Component {

    render() {
        const { modes, currentMode,
                maps, currentMap,
                vehicles, currentVehicle } = this.props;

        return (
            <React.Fragment>
                <Selector name={intl.get('setup_mode')}
                          options={Object.keys(modes).sort()}
                          currentOption={currentMode}
                          onChange={(event) => {
                            console.log(event.target.value);
                            WS.changeSetupMode(event.target.value);
                          }} />
                <Selector name={intl.get('vehicle')}
                          options={vehicles}
                          currentOption={currentVehicle}
                          onChange={(event) => {
                            console.log(event.target.value);
                            WS.changeVehicle(event.target.value);
                          }} />
                <Selector name={intl.get('map')}
                          options={maps}
                          currentOption={currentMap}
                          onChange={(event) => {
                            console.log(event.target.value);
                            WS.changeMap(event.target.value);
                          }} />
            </React.Fragment>
        );
    }
}