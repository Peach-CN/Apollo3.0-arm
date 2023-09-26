/* import "whatwg-fetch";
 * import "font-awesome-webpack";*/

import * as ReactDOM from "react-dom";
import React from "react";
import { Provider } from "mobx-react";

import "styles/main.scss";
import STORE from "store";
import Dreamview from "components/Dreamview";

import intl from 'react-intl-universal';

const locales = {
"en": require('./locales/en-US.json'),
"zh": require('./locales/zh-CN.json'),
};


export default class App extends React.Component {
  componentDidMount() {
    this.loadLocales();
  }

  loadLocales() {
    const language = (navigator.languages && navigator.languages[0]) || navigator.language;
    console.log(language);
    if(!localStorage.getItem("defaultLng")){
      localStorage.setItem('defaultLng', language.split('-')[0]);//默认中文
    }
    intl.init({
      currentLocale: localStorage.getItem("defaultLng"),
      locales,
    })
    .then(() => {
    });
  }
  render() {
    return (
      <Provider store={STORE}>
        <Dreamview />
      </Provider>
    );
  }
}

ReactDOM.render(
  <App />,
  document.getElementById("root")
);
