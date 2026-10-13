"use strict";

importJsOnce("js/viewers/meta/Viewer.js");
importJsOnce("js/viewers/meta/Space2DViewer.js");
importJsOnce("js/viewers/meta/Space3DViewer.js");

importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/ProcessListViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/LaserScanViewer.js");
importJsOnce("js/viewers/GeometryViewer.js");
importJsOnce("js/viewers/PolygonViewer.js");
importJsOnce("js/viewers/DiagnosticViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/PointCloud2Viewer.js");
importJsOnce("js/viewers/ImuViewer.js");
importJsOnce("js/viewers/JointStateViewer.js");
importJsOnce("js/viewers/TwistPublisherViewer.js");
importJsOnce("js/viewers/JointCommandViewer.js");

// GenericViewer must be last
importJsOnce("js/viewers/GenericViewer.js");

importJsOnce("js/transports/WebSocketV1Transport.js");

var snackbarContainer = document.querySelector('#demo-toast-example');

let subscriptions = {};

if(window.localStorage && window.localStorage.subscriptions) {
  if(window.location.search && window.location.search.indexOf("reset") !== -1) {
    subscriptions = {};
    updateStoredSubscriptions();
    window.location.href = "?";
  } else {
    try {
      subscriptions = JSON.parse(window.localStorage.subscriptions);
    } catch(e) {
      console.log(e);
      subscriptions = {};
    }
  }
}

let $grid = null;
$(() => {
  $grid = $('.grid').masonry({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
  $grid.masonry("layout");
});

setInterval(() => {
  if(currentTransport && !currentTransport.isConnected()) {
    console.log("attempting to reconnect ...");
    currentTransport.connect();
  }
}, 5000);

function updateStoredSubscriptions() {
  if(window.localStorage) {
    let storedSubscriptions = {};
    for(let topicName in subscriptions) {
      storedSubscriptions[topicName] = {
        topicType: subscriptions[topicName].topicType,
        viewerName: subscriptions[topicName].viewerName,
      };
    }
    window.localStorage['subscriptions'] = JSON.stringify(storedSubscriptions);
  }
}

function newCard() {
  // creates a new card, adds it to the grid, and returns it.
  let card = $("<div></div>").addClass('card')
    .appendTo($('.grid'));
  return card;
}

let onOpen = function() {
  const urlParams = new URLSearchParams(window.location.search);

  for( let [key, value] of urlParams ){
    key = key.replace(/\\/g, '/');
    value = value.replace(/\\/g, '/');

    console.log("Auto subscribing to " + key + " of type " + value);
      
    const subscriptions = JSON.parse(window.localStorage.getItem('subscriptions') || '{}');
    if (!(key in subscriptions)) {
      initSubscribe({topicName: key, topicType: value});
    }
  }          
  
  for(let topic_name in subscriptions) {
    console.log("Re-subscribing to " + topic_name);
    initSubscribe({topicName: topic_name,
                   topicType: subscriptions[topic_name].topicType,
                   viewerName: subscriptions[topic_name].viewerName});
  }


}

let onSystem = function(system) {
  if(system.hostname) {
    console.log("hostname: " + system.hostname);
    $('.mdl-layout-title').text("ROSboard: " + system.hostname);
  }

  if(system.version) {
    console.log("server version: " + system.version);
    versionCheck(system.version);
  }
}

let onMsg = function(msg) {
  if(!subscriptions[msg._topic_name]) {
    console.log("Received unsolicited message", msg);
  } else if(!subscriptions[msg._topic_name].viewer) {
    console.log("Received msg but no viewer", msg);
  } else {
    subscriptions[msg._topic_name].viewer.update(msg);
  }
}

let currentTopics = {};
let currentTopicsStr = "";

let onTopics = function(topics) {
  
  // check if topics has actually changed, if not, don't do anything
  // lazy shortcut to deep compares, might possibly even be faster than
  // implementing a deep compare due to
  // native optimization of JSON.stringify
  let newTopicsStr = JSON.stringify(topics);
  if(newTopicsStr === currentTopicsStr) return;
  currentTopics = topics;
  currentTopicsStr = newTopicsStr;
  
  let topicTree = treeifyPaths(Object.keys(topics));
  
  $("#topics-nav-ros").empty();
  $("#topics-nav-system").empty();
  
  addTopicTreeToNav(topicTree[0], $('#topics-nav-ros'));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_dmesg", topicType: "rcl_interfaces/msg/Log"}); })
  .text("dmesg")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_top", topicType: "robocore_viz_msgs/msg/ProcessList"}); })
  .text("Processes")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_system_stats", topicType: "robocore_viz_msgs/msg/SystemStats"}); })
  .text("System stats")
  .appendTo($("#topics-nav-system"));

  // Teleop card: publishes geometry_msgs/TwistStamped to /cmd_vel_joy.
  //
  // On the bundled stack (base101_control/config/twist_mux.yaml has
  // use_stamped:=true) twist_mux subscribes its input topics as TwistStamped
  // — match that type here. If your twist_mux is configured with
  // use_stamped:=false, change this to "geometry_msgs/msg/Twist". DDS will
  // refuse the publisher if the type doesn't match what's already on the
  // topic (the backend's pre-check turns that into a clear logwarn instead
  // of an RCLError).
  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initPublishCard({
    topicName: window.ROBOCORE_CONFIG.drive.topic,
    topicType: window.ROBOCORE_CONFIG.drive.type,
    viewerType: TwistPublisherViewer,
  }); })
  .text("Teleop")
  .appendTo($("#topics-nav-system"));

  // Joint sliders card: subscribes /joint_states for live positions and
  // publishes std_msgs/Float64MultiArray position commands to the base101
  // controller topics (tower, both arms, both grippers) — one panel for
  // every controlled joint. Groups whose hardware isn't loaded are hidden.
  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({
    topicName: "/joint_states",
    topicType: "sensor_msgs/msg/JointState",
    viewerName: "JointCommandViewer",
  }); })
  .text("Joint sliders")
  .appendTo($("#topics-nav-system"));
}

function addTopicTreeToNav(topicTree, el, level = 0, path = "") {
  topicTree.children.sort((a, b) => {
    if(a.name>b.name) return 1;
    if(a.name<b.name) return -1;
    return 0;
  });
  topicTree.children.forEach((subTree, i) => {
    let subEl = $('<div></div>')
    .css(level < 1 ? {} : {
      "padding-left": "0pt",
      "margin-left": "12pt",
      "border-left": "1px dashed #808080",
    })
    .appendTo(el);
    let fullTopicName = path + "/" + subTree.name;
    let topicType = currentTopics[fullTopicName];
    if(topicType) {
      $('<a></a>')
        .addClass("mdl-navigation__link")
        .css({
          "padding-left": "12pt",
          "margin-left": 0,
        })
        .click(() => { initSubscribe({topicName: fullTopicName, topicType: topicType}); })
        .text(subTree.name)
        .appendTo(subEl);
    } else {
      $('<a></a>')
      .addClass("mdl-navigation__link")
      .attr("disabled", "disabled")
      .css({
        "padding-left": "12pt",
        "margin-left": 0,
        opacity: 0.5,
      })
      .text(subTree.name)
      .appendTo(subEl);
    }
    addTopicTreeToNav(subTree, subEl, level + 1, path + "/" + subTree.name);
  });
}

// Publisher cards (e.g. teleop) don't subscribe to anything — they create a
// viewer that sends messages OUT via the WS transport's publish() helper.
// Tracked separately from `subscriptions` so we don't try to re-subscribe
// to them on page reload.
let publisherCards = {};
function initPublishCard({topicName, topicType, viewerType}) {
  let key = topicName + "::" + viewerType.name;
  if(publisherCards[key]) {
    // Already open — flash the existing card instead of opening a duplicate.
    let el = publisherCards[key].card;
    if(el && el.length) {
      el[0].scrollIntoView({behavior: "smooth", block: "center"});
      el.css({outline: "2px solid #4caf50"});
      setTimeout(() => el.css({outline: ""}), 600);
    }
    return;
  }
  let card = newCard();
  let instance;
  try {
    instance = new viewerType(card, topicName, topicType);
  } catch(e) {
    console.log(e);
    card.remove();
    return;
  }
  publisherCards[key] = {card: card, viewer: instance, topicName: topicName, topicType: topicType};
  $grid.masonry("appended", card);
  $grid.masonry("layout");
}

function initSubscribe({topicName, topicType, viewerName}) {
  console.log( "Subscribing to " + topicName + " of type " + topicType);
  // creates a subscriber for topicName
  // and also initializes a viewer (if it doesn't already exist)
  // in advance of arrival of the first data
  // this way the user gets a snappy UI response because the viewer appears immediately
  //
  // viewerName (optional): class name of a specific Viewer to use instead of
  // the type's default — e.g. the "Joint sliders" nav entry opens
  // /joint_states with JointCommandViewer. Persisted with the subscription so
  // the same card comes back on reload. If a card already exists for the
  // topic with a different viewer, it gets replaced.
  if(!subscriptions[topicName]) {
    subscriptions[topicName] = {
      topicType: topicType,
    }
  }
  if(viewerName) subscriptions[topicName].viewerName = viewerName;
  currentTransport.subscribe({topicName: topicName});
  let requested = subscriptions[topicName].viewerName ?
    Viewer._viewers.find(v => v.name === subscriptions[topicName].viewerName) : null;
  if(subscriptions[topicName].viewer && requested &&
     !(subscriptions[topicName].viewer instanceof requested)) {
    // viewer switch: drop the old card, fall through to create the new one
    let old = subscriptions[topicName].viewer;
    subscriptions[topicName].viewer = null;
    try { old.destroy(); } catch(e) {}
    try { $grid.masonry("remove", old.card); $grid.masonry("layout"); } catch(e) {}
  }
  if(!subscriptions[topicName].viewer) {
    let card = newCard();
    let viewer = requested || Viewer.getDefaultViewerForType(topicType);
    try {
      subscriptions[topicName].viewer = new viewer(card, topicName, topicType);
    } catch(e) {
      console.log(e);
      card.remove();
    }
    $grid.masonry("appended", card);
    $grid.masonry("layout");
  }
  updateStoredSubscriptions();
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/robocore_viz/v1",
    onOpen: onOpen,
    onMsg: onMsg,
    onTopics: onTopics,
    onSystem: onSystem,
  });
  currentTransport.connect();
}

function treeifyPaths(paths) {
  // turn a bunch of ros topics into a tree
  let result = [];
  let level = {result};

  paths.forEach(path => {
    path.split('/').reduce((r, name, i, a) => {
      if(!r[name]) {
        r[name] = {result: []};
        r.result.push({name, children: r[name].result})
      }
      
      return r[name];
    }, level)
  });
  return result;
}

let lastBotherTime = 0.0;
function versionCheck(currentVersionText) {
  $.get("https://raw.githubusercontent.com/dheera/robocore_viz/release/setup.py").done((data) => {
    let matches = data.match(/version='(.*)'/);
    if(matches.length < 2) return;
    let latestVersion = matches[1].split(".").map(num => parseInt(num, 10));
    let currentVersion = currentVersionText.split(".").map(num => parseInt(num, 10));
    let latestVersionInt = latestVersion[0] * 1000000 + latestVersion[1] * 1000 + latestVersion[2];
    let currentVersionInt = currentVersion[0] * 1000000 + currentVersion[1] * 1000 + currentVersion[2];
    if(currentVersion < latestVersion && Date.now() - lastBotherTime > 1800000) {
      lastBotherTime = Date.now();
      snackbarContainer.MaterialSnackbar.showSnackbar({
        message: "New version of ROSboard available (" + currentVersionText + " -> " + matches[1] + ").",
        actionText: "Check it out",
        actionHandler: ()=> {window.location.href="https://github.com/dheera/rosboard/"},
      });
    }
  });
}

$(() => {
  if(window.location.href.indexOf("robocore_viz.com") === -1) {
    initDefaultTransport();
  }
});

Viewer.onClose = function(viewerInstance) {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;

  // Publisher card (e.g. teleop) — never subscribed to anything, so just
  // detach the card and forget it. No unsubscribe, no localStorage update.
  let pubKey = topicName + "::" + viewerInstance.constructor.name;
  if(publisherCards[pubKey]) {
    try { viewerInstance.destroy(); } catch(e) {}
    $grid.masonry("remove", viewerInstance.card);
    $grid.masonry("layout");
    delete(publisherCards[pubKey]);
    return;
  }

  currentTransport.unsubscribe({topicName:topicName});
  $grid.masonry("remove", viewerInstance.card);
  $grid.masonry("layout");
  delete(subscriptions[topicName].viewer);
  delete(subscriptions[topicName]);
  updateStoredSubscriptions();
}

Viewer.onSwitchViewer = (viewerInstance, newViewerType) => {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  if(!subscriptions[topicName].viewer === viewerInstance) console.error("viewerInstance does not match subscribed instance");
  let card = subscriptions[topicName].viewer.card;
  subscriptions[topicName].viewer.destroy();
  delete(subscriptions[topicName].viewer);
  subscriptions[topicName].viewer = new newViewerType(card, topicName, topicType);
};


