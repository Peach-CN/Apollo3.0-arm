layui.define(['jquery', 'layer'], function (exports) {
    var $ = layui.jquery,
        layer = layui.layer;

    var obj = {
        preserve: function (type) {
            var resultMap = { "type": type };
            if (!window.WebSocket) window.WebSocket = window.MozWebSocket;
            socket.send(JSON.stringify(resultMap));
            if (type == "requestSaveBagFiles") {
                $("#preserve").addClass("layui-btn-disabled").attr('disabled', "true");
            } else if (type == "requestProcessBagFiles") {
                $("#deviationSave").addClass("layui-btn-disabled").attr('disabled', "true");
            } else {
                $("#speedSave").addClass("layui-btn-disabled").attr('disabled', "true");
            }
            layer.alert("The save request has been sent", { icon: 1 });
        }
    };
    exports('preserve', obj);
});