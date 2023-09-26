import React from "react";
import { inject, observer } from "mobx-react";

import DataRecorder from "components/DataRecorder";
import ModuleController from "components/ModuleController";
import Menu from "components/SideBar/Menu";
import POI from "components/SideBar/POI";
import Tasks from "components/Tasks";
import SetupWizard from "components/SetupWizard";

@inject("store") @observer
export default class ToolView extends React.Component {
    render() {
        const { options, routeEditingManager, hmi, newDisengagementReminder } = this.props.store;

        return (
            <div className="tools">
                {(!hmi.showSetupWizard && options.showTasks) && <Tasks />}
                {(!hmi.showSetupWizard && options.showModuleController) && <ModuleController />}
                {(!hmi.showSetupWizard && options.showMenu) && <Menu options={options} />}
                {(!hmi.showSetupWizard && options.showPOI) && (
                    <POI
                        routeEditingManager={routeEditingManager}
                        options={options}
                        inNavigationMode={hmi.inNavigationMode}
                    />
                )}
                {(!hmi.showSetupWizard && options.showDataRecorder) && (
                    <DataRecorder
                        newDisengagementReminder={newDisengagementReminder}
                    />
                )}
                {hmi.showSetupWizard && <SetupWizard />}
            </div>
        );
    }
}
