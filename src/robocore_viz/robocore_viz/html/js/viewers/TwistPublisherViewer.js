"use strict";

// TwistPublisherViewer
// --------------------
// A teleop card. Unlike every other viewer in robocore_viz, this one *publishes*
// to ROS instead of subscribing — it sends geometry_msgs/Twist messages to a
// configurable topic (default /cmd_vel_joy) over the existing robocore_viz WS via
// MSG_PUB. The bundled backend gates the publish on a configurable allowlist
// of message types.
//
// The card has a virtual joystick (touch + mouse), a speed slider that scales
// both axes, and an E-stop button. ~20 Hz updates while the joystick is held;
// release → zero command immediately. The backend has its own watchdog that
// will also zero the topic if the client falls silent.

class TwistPublisherViewer extends Viewer {

  onCreate() {
    super.onCreate();

    this.maxLinear = 1.0;   // m/s at slider=100
    this.maxAngular = 2.0;  // rad/s at slider=100
    this.sendRateHz = 20;

    this.card.title.text("Teleop  →  " + this.topicName);

    // We're a publisher, not a subscriber. The base Viewer constructor adds a
    // "waiting for data" spinner *after* onCreate() returns, so we have to
    // wait one tick before yanking it — otherwise this.loaderContainer is
    // still undefined here and the spinner gets added unchallenged.
    setTimeout(() => {
      if(this.loaderContainer) {
        this.loaderContainer.remove();
        this.loaderContainer = null;
      }
    }, 0);

    let body = $('<div></div>').css({
      "padding": "12pt",
      "display": "grid",
      "grid-template-columns": "1fr 80px",
      "gap": "10pt",
      "min-height": "280px",
    }).appendTo(this.card.content);

    // --- joystick pad -----------------------------------------------------
    this.pad = $('<div></div>').css({
      "position": "relative",
      "background": "#262626",
      "border-radius": "10pt",
      "overflow": "hidden",
      "touch-action": "none",
      "min-height": "260px",
      "user-select": "none",
    }).appendTo(body);

    // crosshair
    $('<div></div>').css({
      "position": "absolute", "inset": "0",
      "background-image":
        "linear-gradient(to right,  transparent 49.7%, #3a3a3a 49.7% 50.3%, transparent 50.3%)," +
        "linear-gradient(to bottom, transparent 49.7%, #3a3a3a 49.7% 50.3%, transparent 50.3%)",
      "pointer-events": "none",
    }).appendTo(this.pad);

    this.stick = $('<div></div>').css({
      "position": "absolute", "left": "50%", "top": "50%",
      "width": "22%", "height": "22%",
      "min-width": "56px", "min-height": "56px",
      "max-width": "120px", "max-height": "120px",
      "transform": "translate(-50%, -50%)",
      "border-radius": "50%",
      "background": "radial-gradient(circle at 30% 30%, #5a6878, #2a3038)",
      "box-shadow": "0 4px 14px rgba(0,0,0,0.4), inset 0 0 0 1px #4a5360",
      "transition": "transform 80ms ease-out",
      "pointer-events": "none",
    }).appendTo(this.pad);

    this.readout = $('<div></div>').css({
      "position": "absolute", "left": "10px", "bottom": "8px",
      "font-family": "'JetBrains Mono', monospace",
      "font-size": "10px",
      "color": "#888",
      "pointer-events": "none",
      "white-space": "pre",
    }).text("lin  0.00 m/s\nang  0.00 rad/s").appendTo(this.pad);

    // --- speed slider + e-stop column ------------------------------------
    let col = $('<div></div>').css({
      "display": "flex", "flex-direction": "column",
      "align-items": "center", "gap": "8px",
    }).appendTo(body);

    let speedBox = $('<div></div>').css({
      "flex": "1", "min-height": "180px",
      "background": "#262626", "border-radius": "10pt",
      "padding": "10px 6px",
      "display": "flex", "flex-direction": "column",
      "align-items": "center", "width": "100%",
    }).appendTo(col);

    $('<div></div>').css({
      "font-size": "10px", "color": "#888",
      "letter-spacing": "0.08em", "text-transform": "uppercase",
      "margin-bottom": "6px",
    }).text("speed").appendTo(speedBox);

    this.speed = $('<input type="range" min="10" max="100" value="60" step="1">').css({
      "-webkit-appearance": "slider-vertical",
      "appearance": "slider-vertical",
      "writing-mode": "vertical-lr",
      "direction": "rtl",
      "width": "24px",
      "flex": "1",
      "background": "transparent",
    }).appendTo(speedBox);

    this.speedValue = $('<div></div>').css({
      "font-family": "'JetBrains Mono', monospace",
      "font-size": "12px", "color": "#cfcfcf",
      "margin-top": "6px",
    }).text("60%").appendTo(speedBox);

    this.speed.on('input', () => {
      this.speedValue.text(this.speed.val() + "%");
    });

    this.estop = $('<button>STOP</button>').css({
      "width": "100%", "padding": "14px 0",
      "border": "0", "border-radius": "10pt",
      "background": "#ff4d4f", "color": "white",
      "font-weight": "700", "letter-spacing": "0.08em",
      "font-size": "12px",
      "box-shadow": "0 3px 12px rgba(255,77,79,0.35)",
      "cursor": "pointer",
    }).appendTo(col);
    this.estop.on('click', () => {
      this.cur = {x: 0, y: 0};
      this._updateStick();
      this._sendCurrent();
    });

    // --- pointer handling -------------------------------------------------
    this.cur = {x: 0, y: 0};      // -1..1 in pad-local coords
    this.active = false;
    this.pointerId = null;

    this.pad.on('pointerdown', (ev) => this._start(ev.originalEvent));
    this.pad.on('pointermove', (ev) => this._move(ev.originalEvent));
    this.pad.on('pointerup',     (ev) => this._end(ev.originalEvent));
    this.pad.on('pointercancel', (ev) => this._end(ev.originalEvent));
    this.pad.on('lostpointercapture', (ev) => this._end(ev.originalEvent));

    // resend at ~20 Hz so the backend watchdog stays happy on small blips
    this._interval = setInterval(() => {
      if(this.active) this._sendCurrent();
    }, Math.round(1000 / this.sendRateHz));

    // background tab / hidden page → stop
    this._visHandler = () => {
      if(document.hidden) {
        this.cur = {x: 0, y: 0};
        this._updateStick();
        this._sendCurrent();
      }
    };
    document.addEventListener('visibilitychange', this._visHandler);
  }

  _padGeom() {
    let el = this.pad[0].getBoundingClientRect();
    let r = Math.min(el.width, el.height) * 0.42;
    return {cx: el.left + el.width / 2, cy: el.top + el.height / 2, r: r};
  }

  _setFromEvent(ev) {
    let g = this._padGeom();
    let dx = (ev.clientX - g.cx) / g.r;
    let dy = (ev.clientY - g.cy) / g.r;
    let m = Math.hypot(dx, dy);
    if(m > 1) { dx /= m; dy /= m; }
    this.cur.x = dx; this.cur.y = dy;
  }

  _start(ev) {
    if(this.pointerId !== null) return;
    this.pointerId = ev.pointerId;
    this.pad[0].setPointerCapture(this.pointerId);
    this.active = true;
    this._setFromEvent(ev);
    this._updateStick();
    this._sendCurrent();
  }
  _move(ev) {
    if(ev.pointerId !== this.pointerId) return;
    this._setFromEvent(ev);
    this._updateStick();
  }
  _end(ev) {
    if(ev.pointerId !== this.pointerId) return;
    try { this.pad[0].releasePointerCapture(this.pointerId); } catch(e) {}
    this.pointerId = null;
    this.active = false;
    this.cur = {x: 0, y: 0};
    this._updateStick();
    this._sendCurrent();
  }

  _updateStick() {
    let g = this._padGeom();
    let dx = this.cur.x * g.r;
    let dy = this.cur.y * g.r;
    this.stick.css('transform',
      'translate(calc(-50% + ' + dx + 'px), calc(-50% + ' + dy + 'px))');
  }

  _currentTwist() {
    let s = Number(this.speed.val()) / 100;
    // up = forward; right stick = turn right (negative yaw, ROS convention)
    let linear  = -this.cur.y * this.maxLinear  * s;
    let angular = -this.cur.x * this.maxAngular * s;
    return {linear: linear, angular: angular};
  }

  _sendCurrent() {
    let t = this._currentTwist();
    this.readout.text(
      "lin  " + t.linear .toFixed(2).padStart(5) + " m/s\n" +
      "ang  " + t.angular.toFixed(2).padStart(5) + " rad/s"
    );
    if(typeof currentTransport === 'undefined' || !currentTransport || !currentTransport.publish) return;

    // Shape the payload to whichever message type this card is configured for.
    // The Jazzy chain (robocore_viz → twist_mux use_stamped:=true → diff_drive_controller)
    // wants TwistStamped end-to-end; the plain-Twist variant is still useful
    // when publishing direct into an older controller / a custom subscriber.
    let inner = {
      linear:  {x: t.linear, y: 0.0, z: 0.0},
      angular: {x: 0.0,      y: 0.0, z: t.angular},
    };
    let msg;
    if(this.topicType === "geometry_msgs/msg/TwistStamped") {
      msg = {header: {frame_id: this.frameId || ""}, twist: inner};
    } else {
      msg = inner;
    }
    currentTransport.publish({
      topicName: this.topicName,
      topicType: this.topicType,
      msg: msg,
    });
  }

  destroy() {
    if(this._interval) clearInterval(this._interval);
    if(this._visHandler) document.removeEventListener('visibilitychange', this._visHandler);
    super.destroy();
  }
}

TwistPublisherViewer.friendlyName = "Teleop (Twist publisher)";
// Intentionally NOT registered for any auto-discovery type:
// we don't want a user who subscribes to a /cmd_vel topic in the nav drawer
// to get this writer-card instead of a normal viewer. The card is constructed
// directly by initPublishCard() when the user clicks the Teleop nav entry.
TwistPublisherViewer.supportedTypes = [];
TwistPublisherViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(TwistPublisherViewer);
