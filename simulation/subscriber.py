"""Send PC/status over Zenoh and log other text messages."""

import json
import logging
import os
import time

import zenoh


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ROS_TELEMETRY_KEYS = {"real_uav/imu", "real_uav/odom"}


def start_zenoh():
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(os.getenv("ZENOH_MODE", "client")))
    config.insert_json5(
        "connect/endpoints",
        json.dumps([os.getenv("ZENOH_PEER", "tcp/127.0.0.1:7447")]),
    )

    while True:
        try:
            session = zenoh.open(config)
        except Exception as error:
            logger.warning("En attente du bridge : %s", error)
            time.sleep(2)
        else:
            logger.info("Session Zenoh démarrée")
            return session


def listener(sample):
    key = str(sample.key_expr).lstrip("/")
    if key in ROS_TELEMETRY_KEYS:
        return  # The ROS 2 bridge handles these messages.

    try:
        payload = sample.payload.to_string()
    except Exception:
        logger.info("Reçu sur '%s' : %d octets", key, len(sample.payload.to_bytes()))
    else:
        logger.info("Reçu sur '%s' : %s", key, payload)


def main():
    session = None
    subscriber = None
    status_publisher = None

    try:
        session = start_zenoh()
        subscriber = session.declare_subscriber("**", listener)
        status_publisher = session.declare_publisher("PC/status")
        logger.info("Écoute des messages Zenoh")

        while True:
            status_publisher.put("PC actif")
            logger.info("Envoyé sur 'PC/status' : PC actif")
            time.sleep(5)

    except KeyboardInterrupt:
        logger.info("Arrêt demandé")
    except Exception:
        logger.exception("Erreur Zenoh")
        raise
    finally:
        if subscriber is not None:
            subscriber.undeclare()
        if status_publisher is not None:
            status_publisher.undeclare()
        if session is not None:
            session.close()


if __name__ == "__main__":
    main()
